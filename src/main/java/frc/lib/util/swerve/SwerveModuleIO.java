package frc.lib.util.swerve;

import org.littletonrobotics.junction.AutoLog;

/** IO Class for SwerveModule */
public interface SwerveModuleIO {
    /** Inputs Class for SwerveModule */
    @AutoLog
    public static class SwerveModuleInputs {
        public double driveMotorSelectedPosition;
        public double driveMotorSelectedSensorVelocity;
        public double angleMotorSelectedPosition;
        public double absolutePositionAngleEncoder;
        public double[] odometryTimestamps;
        // public double driveMotorTemp;
        // public double angleMotorTemp;
    }

    public default void setModNumber(int number) {}

    public default void updateInputs(SwerveModuleInputs inputs) {}

    public default void setDriveMotor(double mps) {}

    public default void setDriveMotorPower(double power) {}

    public default void setAngleMotor(double angle) {}

    public default void setAngleSelectedSensorPosition(double angle) {}

    public default void setPositionAngleMotor(double absolutePosition) {}

}
