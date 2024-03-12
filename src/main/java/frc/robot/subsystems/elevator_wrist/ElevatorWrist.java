package frc.robot.subsystems.elevator_wrist;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.OperatorState;
import frc.robot.RobotContainer;

/**
 * Elevator and Wrist Subsystem
 */
public class ElevatorWrist extends SubsystemBase {
    public ElevatorWristIO io;
    public ElevatorWristInputsAutoLogged inputs = new ElevatorWristInputsAutoLogged();
    private CommandXboxController operator;

    private boolean pidEnabled = false;

    ProfiledPIDController elevatorPIDController =
        new ProfiledPIDController(Constants.ElevatorWristConstants.PID.ELEVATOR_KP,
            Constants.ElevatorWristConstants.PID.ELEVATOR_KI,
            Constants.ElevatorWristConstants.PID.ELEVATOR_KD,
            new TrapezoidProfile.Constraints(
                Constants.ElevatorWristConstants.PID.ELEVATOR_MAX_VELOCITY,
                Constants.ElevatorWristConstants.PID.ELEVATOR_MAX_ACCELERATION));

    PIDController wristPIDController =
        new PIDController(Constants.ElevatorWristConstants.PID.WRIST_KP,
            Constants.ElevatorWristConstants.PID.WRIST_KI,
            Constants.ElevatorWristConstants.PID.WRIST_KD);

    PIDController wristProfiledPIDController =
        new PIDController(Constants.ElevatorWristConstants.PID.WRIST_LARGE_KP,
            Constants.ElevatorWristConstants.PID.WRIST_KI,
            Constants.ElevatorWristConstants.PID.WRIST_KD);

    private InterpolatingDoubleTreeMap radiusToAngle = new InterpolatingDoubleTreeMap();

    private GenericEntry wristAngle = RobotContainer.mainDriverTab.add("Wrist Angle", 0)
        .withWidget("Radial Gauge").withProperties(Map.of("min_value", -180, "max_value", 180))
        .withPosition(13, 0).withSize(4, 4).getEntry();

    private GenericEntry elevatorHeight = RobotContainer.mainDriverTab.add("Elevator Height", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min_value", Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            "max_value", Constants.ElevatorWristConstants.SetPoints.MAX_EXTENSION, "orientation",
            "vertical"))
        .withPosition(8, 2).withSize(2, 2).getEntry();

    /**
     * Create new ElevatorWrist.
     */
    public ElevatorWrist(ElevatorWristIO io, CommandXboxController operator) {
        this.operator = operator;
        this.io = io;
        io.updateInputs(inputs);
        wristPIDController
            .setSetpoint(Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE.getRotations());
        wristPIDController.setTolerance(Rotation2d.fromDegrees(0.1).getRotations());
        elevatorPIDController.setGoal(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT);
        wristPIDController.setIZone(Rotation2d.fromDegrees(5).getRotations());
        wristProfiledPIDController.setIZone(Rotation2d.fromDegrees(1).getRotations());
        // radiusToAngle.put(4.32, 33.43);
        // radiusToAngle.put(4.21, 34.11);
        // radiusToAngle.put(3.99, 34.0);
        // radiusToAngle.put(2.59, 40.5);
        // radiusToAngle.put(1.72, 49.25);
        radiusToAngle.put(Units.inchesToMeters(92), 43.2);
        radiusToAngle.put(Units.inchesToMeters(98), 42.0);
        radiusToAngle.put(Units.inchesToMeters(134), 39.16);
        radiusToAngle.put(Units.inchesToMeters(144), 36.13);
        radiusToAngle.put(Units.inchesToMeters(163), 32.41);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorWrist", inputs);

        if (inputs.wristAbsoluteEncRawValue > 0.5) {
            inputs.wristAbsoluteEncRawValue -= 1.0;
        }

        if (OperatorState.manualModeEnabled()) {
            elevatorPIDController.setGoal(
                elevatorPIDController.getGoal().position + operator.getRightY() * 4.0 * 0.02);
            wristPIDController.setSetpoint(wristProfiledPIDController.getSetpoint()
                + operator.getLeftY() * Rotation2d.fromDegrees(5.0).getRotations() * 0.02);
        }

        wristProfiledPIDController.setSetpoint(wristPIDController.getSetpoint());

        // double elevatorPIDValue = elevatorPIDController.calculate(elevatorDistanceTraveled());

        // double elevatorFeedForwardValue =
        // elevatorFeedForward.calculate(0, 0, elevatorPIDController.getPeriod());

        // double wristFeedForwardValue =
        // wristFeedForward.calculate(0, 0, wristPIDController.getPeriod());

        // if (inputs.topLimitSwitch && elevatorPIDValue > 0) {
        // elevatorPIDValue = 0;
        // }

        // if (inputs.bottomLimitSwitch && elevatorPIDValue < 0) {
        // elevatorPIDValue = 0;
        // }

        Rotation2d calculatedWristAngle = getWristAngle();

        double calculatedHeight = getHeight();

        wristAngle.setDouble(calculatedWristAngle.getDegrees());
        elevatorHeight.setDouble(calculatedHeight);

        double wristPIDValue = wristPIDController.calculate(calculatedWristAngle.getRotations());

        double profiledWristPIDValue =
            wristProfiledPIDController.calculate(calculatedWristAngle.getRotations());
        // SmartDashboard.putNumber("wristError",
        // Rotation2d.fromRotations(wristPIDController.getPositionError()).getDegrees());
        if (Math.abs(wristPIDController.getPositionError()) > Rotation2d.fromDegrees(5)
            .getRotations()) {
            wristPIDValue = profiledWristPIDValue;
        }

        double elevatorPIDValue = elevatorPIDController.calculate(calculatedHeight);
        double elevatorFeedForward = 0.4;

        io.setElevatorVoltage(-elevatorFeedForward - elevatorPIDValue);
        io.setWristVoltage(-wristPIDValue);

        Logger.recordOutput("/ElevatorWrist/Wrist/PID Voltage", elevatorPIDValue);
        Logger.recordOutput("/ElevatorWrist/Wrist/PID setpoint",
            elevatorPIDController.getSetpoint().position);

        SmartDashboard.putNumber("Wrist Goal",
            Rotation2d.fromRotations(wristPIDController.getSetpoint()).getDegrees());
        SmartDashboard.putNumber("Wrist Profiled Goal",
            Rotation2d.fromRotations(wristProfiledPIDController.getSetpoint()).getDegrees());
        SmartDashboard.putNumber("Elevator PID Voltage", elevatorPIDValue);
        SmartDashboard.putNumber("Wrist PID Voltage", wristPIDValue);
        SmartDashboard.putNumber("ElevatorWrist PID setpoint",
            elevatorPIDController.getSetpoint().position);
        SmartDashboard.putNumber("ElevatorWrist Elevator Encoder Value",
            inputs.elevatorRelativeEncRawValue);
        SmartDashboard.putNumber("ElevatorWrist Wrist Encoder Value",
            inputs.wristAbsoluteEncRawValue);
        SmartDashboard.putNumber("ElevatorWrist Amp Drawn", inputs.wristMotorAmp);
        // Logger.recordOutput("/ElevatorWrist/Wrist/Feedforward", wristFeedForwardValue);
        // Logger.recordOutput("/ElevatorWrist/Wrist/Combined Voltage",
        // wristFeedForwardValue + wristPIDValue);
        // Logger.recordOutput("/ElevatorWrist/Wrist/Combined Voltage", wristPIDValue);

    }

    /**
     * Calculate elevator height from raw encoder value.
     */
    public double getHeight() {
        return Constants.ElevatorWristConstants.ELEVATOR_M * inputs.elevatorRelativeEncRawValue
            + Constants.ElevatorWristConstants.ELEVATOR_B;
    }

    /**
     * Calculate wrist angle from raw encoder value.
     */
    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(
            Constants.ElevatorWristConstants.WRIST_M * inputs.wristAbsoluteEncRawValue
                + Constants.ElevatorWristConstants.WRIST_B);
    }

    /**
     * Determine the angle of wrist based on the distance from the shooter
     *
     * @param position Position of the robot
     * @return Rotation of the wrist
     */
    public Rotation2d getAngleFromDistance(Pose2d position) {
        Pose2d speakerPos =
            FieldConstants.allianceFlip(FieldConstants.Speaker.centerSpeakerOpening);
        double distFromSpeaker =
            position.getTranslation().minus(speakerPos.getTranslation()).getNorm();
        SmartDashboard.putNumber("Dist from speaker", distFromSpeaker);
        return Rotation2d.fromDegrees(radiusToAngle.get(distFromSpeaker));
    }

    /**
     * Set the Wrist to a specified angle
     *
     * @param angle Angle of the wrist
     */
    public void setWristAngle(Rotation2d angle) {
        wristPIDController.setSetpoint(angle.getRotations());
        wristProfiledPIDController.setSetpoint(angle.getRotations());
        pidEnabled = true;
    }

    /**
     * Set elevator and wrist to amp position. Performs two steps to avoid colliding with
     * electronics box.
     */
    public Command ampPosition() {
        return goToPosition(Constants.ElevatorWristConstants.SetPoints.AMP_HEIGHT,
            Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE).until(() -> getHeight() > 32)
                .withTimeout(2)
                .andThen(goToPosition(Constants.ElevatorWristConstants.SetPoints.AMP_HEIGHT,
                    Constants.ElevatorWristConstants.SetPoints.AMP_ANGLE).withTimeout(2));
    }

    /**
     * Set elevator and wrist to amp position. Performs two steps to avoid colliding with
     * electronics box.
     */
    public Command climbPosition() {
        return goToPosition(Constants.ElevatorWristConstants.SetPoints.CLIMBING_HEIGHT,
            Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE).until(() -> getHeight() > 32)
                .withTimeout(2)
                .andThen(goToPosition(Constants.ElevatorWristConstants.SetPoints.CLIMBING_HEIGHT,
                    Constants.ElevatorWristConstants.SetPoints.CLIMBING_ANGLE).withTimeout(2));
    }

    /**
     * Set elevator to home position which can fit under the stage. Performs two steps to avoid
     * colliding with electronics box.
     */
    public Command homePosition() {
        Command checkHome = Commands.either(
            goToPosition(36, Rotation2d.fromDegrees(24))
                .until(() -> getWristAngle().getDegrees() > 15).withTimeout(2),
            Commands.none(), () -> !elevatorAtHome());
        Command goHome = goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE).withTimeout(2);
        return checkHome.andThen(goHome);
    }

    /**
     * Command to move Elevator and Wrist to set positions
     *
     * @param height The height of the elevator in meters
     * @param angle The angle of the wrist in {@link Rotation2d}
     *
     * @return A {@link Command}
     */
    public Command goToPosition(double height, Rotation2d angle) {
        return Commands.runOnce(() -> {
            elevatorPIDController.setGoal(height);
            wristPIDController.setSetpoint(angle.getRotations());
            wristProfiledPIDController.setSetpoint(angle.getRotations());
            pidEnabled = true;
        }).andThen(Commands.waitUntil(() -> atGoal()))
            .andThen(Commands.runOnce(() -> pidEnabled = false));
    }

    /**
     * Command to continuously move the Elevator and Wrist to an ever changing position
     *
     * @param height A {@link DoubleSupplier} to provide the height of the elevator in meters
     * @param angle A {@link Supplier} of {@link Rotation2d} of angle of the wrist in
     *        {@link Rotation2d}
     *
     * @return A {@link Command}
     */
    public Command followPosition(DoubleSupplier height, Supplier<Rotation2d> angle) {
        return Commands.run(() -> {
            elevatorPIDController.setGoal(height.getAsDouble());
            wristPIDController.setSetpoint(angle.get().getRotations());
            wristProfiledPIDController.setSetpoint(angle.get().getRotations());
        }).beforeStarting(() -> pidEnabled = true).finallyDo(() -> pidEnabled = false);
    }

    /**
     * Get the height in meters of the elevator based on the rotations of the motor
     *
     * @return Height of elevator in meters
     */
    public double elevatorDistanceTraveled() {
        return inputs.elevatorRelativeEncRawValue
            * Constants.ElevatorWristConstants.SetPoints.LINEAR_DISTANCE;
    }

    /**
     * Get if at elevator + wrist PID goal
     *
     * @return boolean representing if the elevator and wrist PID controllers are at their goals
     */
    public Boolean atGoal() {
        // return elevatorPIDController.atGoal() && wristPIDController.atGoal();
        return wristPIDController.atSetpoint();
        // return true;
    }

    /**
     * Check if the elevator is at the home position
     *
     * @return True if the elevator is home
     */
    public boolean elevatorAtHome() {
        return MathUtil.isNear(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT, getHeight(),
            6);
    }

}
