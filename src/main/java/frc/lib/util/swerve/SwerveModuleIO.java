package frc.lib.util.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** IO Class for SwerveModule */
public interface SwerveModuleIO {
    /** Inputs Class for SwerveModule */
    @AutoLog
    public static class SwerveModuleInputs {
        public Angle driveMotorSelectedPosition;
        public AngularVelocity driveMotorSelectedSensorVelocity;
        public Angle angleMotorSelectedPosition;
        public Angle absolutePositionAngleEncoder;
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
