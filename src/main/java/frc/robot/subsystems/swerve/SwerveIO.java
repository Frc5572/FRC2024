package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;

/** IO Class for Swerve */
public interface SwerveIO {

    /** Inputs Class for Swerve */

    public static class SwerveInputs implements LoggableInputs, Cloneable {

        public float yaw;
        public float roll;
        public float pitch;

        public SwerveInputs() {

        }

        @Override
        public void toLog(LogTable table) {
            table.put("Yaw", yaw);
            table.put("Roll", roll);
            table.put("Pitch", pitch);

        }

        @Override
        public void fromLog(LogTable table) {
            yaw = table.get("Yaw", yaw);
            roll = table.get("Roll", roll);
            pitch = table.get("Roll", pitch);
        }


        /**
         * Returns copy of SwerveInputs for LoggableInputs class
         *
         * @return copy of all SwerveInputs
         */
        public SwerveInputs clone() {
            SwerveInputs copy = new SwerveInputs();
            copy.yaw = this.yaw;
            copy.roll = this.roll;
            copy.pitch = this.pitch;
            return copy;
        }
    }

    public default void updateInputs(SwerveInputs inputs, Pose2d previousPose) {}

    /** Instantiating SwerveModules */
    public default SwerveModule createSwerveModule(int moduleNumber, int driveMotorID,
        int angleMotorID, int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleIO() {});
    }

    public default Optional<Pose2d> getInitialPose() {
        return Optional.empty();
    }

    public default void update(int i, Pose2d pose) {}

}
