package frc.robot.subsystems.swerve.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveSetpointGenerator;
import frc.lib.util.SwerveSetpointGenerator.SwerveSetpoint;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;
import frc.robot.subsystems.swerve.mod.ModuleConfig;
import frc.robot.subsystems.swerve.mod.SwerveModule;
import frc.robot.subsystems.swerve.mod.SwerveModuleAngleIO;
import frc.robot.subsystems.swerve.mod.SwerveModuleDriveIO;

/** Swerve Subsystem */
public final class Swerve extends SubsystemBase {

    public static final Lock odometryLock = new ReentrantLock();

    private final SwerveIO io;
    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    private final SwerveModule[] modules;
    private final Alert gyroDisconnectedAlert =
        new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(Constants.Swerve.config.getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics,
        rawGyroRotation, lastModulePositions, new Pose2d());

    private SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(kinematics,
        getMaxLinearSpeedMetersPerSec(),
        Constants.Swerve.config.moduleConstants.maxDriveRate.in(Units.MetersPerSecondPerSecond),
        Constants.Swerve.config.moduleConstants.maxSteerRate.in(Units.RadiansPerSecond));
    private SwerveSetpoint prevSetpoint;

    private Rotation2d fieldOffset = new Rotation2d();

    /** Swerve Subsystem */
    public Swerve(SwerveIO io,
        BiFunction<Integer, ModuleConfig, Pair<SwerveModuleAngleIO, SwerveModuleDriveIO>> modFunc) {
        super("Swerve");
        this.io = io;
        var modules = Constants.Swerve.config.modules();
        this.modules = new SwerveModule[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var modIO = modFunc.apply(i, modules[i]);
            this.modules[i] = new SwerveModule(modules[i], modIO.getFirst(), modIO.getSecond());
        }
        prevSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();
    }

    public Swerve(SwerveIO io,
        BiFunction<Integer, ModuleConfig, SwerveModuleAngleIO> angleFunc,
        BiFunction<Integer, ModuleConfig, SwerveModuleDriveIO> driveFunc) {
        this(io, (i, conf) -> Pair.of(angleFunc.apply(i, conf), driveFunc.apply(i, conf)));
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);
        for (var module : modules) {
            module.updateInputs();
        }
        odometryLock.unlock();

        for (var module : modules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }

            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        double[] sampleTimestamps =
            modules[0].driveInputs.odometryTimestamps; // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                            - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        if (inputs.connected) {
            poseEstimator.resetRotation(inputs.yawPosition);
        }
        // Update gyro alert
        gyroDisconnectedAlert.set(!inputs.connected);
    }

    private void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean useSetpointGenerator) {
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getFieldRelativeHeading());
        }
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates;
        if (useSetpointGenerator) {
            prevSetpoint = setpointGenerator.getFeasibleSetpoint(prevSetpoint, discreteSpeeds,
                Constants.Swerve.config.scrubLimit.in(Units.MetersPerSecond));
            setpointStates = prevSetpoint.moduleStates;
        } else {
            setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,
                getMaxLinearSpeedMetersPerSec());
        }

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public Command drive(Supplier<ChassisSpeeds> speeds, boolean fieldRelative,
        boolean useSetpointGenerator) {
        return this.run(() -> {
            this.drive(speeds.get(), fieldRelative, useSetpointGenerator);
        });
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public Command runCharacterization(DoubleSupplier outputSupplier) {
        return this.run(() -> {
            double output = outputSupplier.getAsDouble();
            for (int i = 0; i < 4; i++) {
                modules[i].runCharacterization(output);
            }
        });
    }

    /** Stops the drive. */
    public Command stop() {
        return this.runOnce(() -> {
            this.drive(new ChassisSpeeds(), false, true);
        });
    }

    public Command resetFieldRelativeOffset() {
        return this.runOnce(() -> {
            fieldOffset = inputs.yawPosition;
        });
    }

    @AutoLogOutput(key = "Swerve/Heading")
    private Rotation2d getFieldRelativeHeading() {
        return inputs.yawPosition.minus(fieldOffset);
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return Constants.Swerve.config.maxLinearSpeed.in(Units.MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec()
            / Constants.Swerve.config.getDriveBaseRadius().in(Units.Meters);
    }
}
