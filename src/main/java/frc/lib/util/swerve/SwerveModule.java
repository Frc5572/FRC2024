package frc.lib.util.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * Swerve Module Subsystem
 */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    // private double lastAngle;

    private SwerveModuleIO io;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    /**
     * Swerve Module
     *
     * @param moduleNumber Module Number
     * @param angleOffset Angle Offset of the CANCoder to align the wheels
     */
    public SwerveModule(int moduleNumber, Rotation2d angleOffset, SwerveModuleIO io) {
        this.io = io;

        this.moduleNumber = moduleNumber;

        this.angleOffset = angleOffset;

        // lastAngle = getState().angle.getDegrees();
        io.updateInputs(inputs);
        resetToAbsolute();
        Logger.processInputs("SwerveModule" + moduleNumber, inputs);
    }

    /**
     * Update inputs for a Swerve Module.
     */
    public void periodic() {
        Robot.profiler.push("updateInputs");
        io.updateInputs(inputs);
        Robot.profiler.swap("processInputs");
        Logger.processInputs("SwerveModule" + moduleNumber, inputs);
        Robot.profiler.pop();
    }

    /**
     * Set the desired state of the Swerve Module
     *
     * @param desiredState The desired {@link SwerveModuleState} for the module
     * @param isOpenLoop Whether the state should be open or closed loop controlled
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        io.setAngleMotor(desiredState.angle.getRotations());
        setSpeed(desiredState, isOpenLoop);
        SmartDashboard.putNumber("desired state speed/" + moduleNumber,
            desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("desired state angle/" + moduleNumber,
            desiredState.angle.getDegrees());
    }

    /**
     * Set the velocity or power of the drive motor
     *
     * @param desiredState The desired {@link SwerveModuleState} of the module
     * @param isOpenLoop Whether the state should be open or closed loop controlled
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double power = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            io.setDriveMotorPower(power);
        } else {
            io.setDriveMotor(desiredState.speedMetersPerSecond);
        }
    }

    /**
     * Get the rotation of the CANCoder
     *
     * @return The rotation of the CANCoder in {@link Rotation2d}
     */
    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(inputs.absolutePositionAngleEncoder);
    }

    /**
     * Reset the Swerve Module angle to face forward
     */
    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        io.setPositionAngleMotor(absolutePosition);
        inputs.angleMotorSelectedPosition = absolutePosition;
    }

    /**
     * Get the current Swerve Module State
     *
     * @return The current {@link SwerveModuleState}
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.rotationPerSecondToMetersPerSecond(inputs.driveMotorSelectedSensorVelocity,
                Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(inputs.angleMotorSelectedPosition));
    }

    /**
     * Get the current Swerve Module Position
     *
     * @return The current {@link SwerveModulePosition}
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(inputs.driveMotorSelectedPosition,
                Constants.Swerve.wheelCircumference),
            Rotation2d.fromRotations(inputs.angleMotorSelectedPosition));

    }
}
