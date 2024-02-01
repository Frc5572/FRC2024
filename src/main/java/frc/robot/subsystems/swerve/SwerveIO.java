package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;

/** IO Class for Swerve */
public interface SwerveIO {

    /** Inputs Class for Swerve */
    @AutoLog
    public static class SwerveInputs {
        public float yaw;
        public float roll;
        public double[] latencies;
        public PhotonPipelineResult[] results;
        public Optional<Pose3d>[] positions;
        public boolean[] seesTarget;
        public Optional<EstimatedRobotPose>[] estimatedRobotPose;
    }

    public default void updateInputs(SwerveInputs inputs, Pose2d previousPose) {}

    /**
     * @param prevEstimatedRobotPose The current best guess at robot pose
     *
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
     *         create the estimate
     */
    public default Optional<EstimatedRobotPose> getFrontLeftEstimatedGlobalPose() {
        return null;
    }

    /** Instantiating SwerveModules */
    public default SwerveModule createSwerveModule(int moduleNumber, int driveMotorID,
        int angleMotorID, int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleIO() {});
    }

    public default Optional<Pose2d> getInitialPose() {
        return null;
    }

    public default void update(int i, Pose2d pose) {}

}
