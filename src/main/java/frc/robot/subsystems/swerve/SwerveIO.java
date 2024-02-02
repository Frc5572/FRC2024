package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;

/** IO Class for Swerve */
public interface SwerveIO {

    /** Inputs Class for Swerve */

    public static class SwerveInputs implements LoggableInputs, Cloneable {

        public SwerveInputs() {

        }

        @Override
        public void toLog(LogTable table) {
            table.put("Yaw", yaw);
            table.put("Roll", roll);
            table.put("Latencies", latencies);
            table.put("Results/1", PhotonPipelineResult.proto, results[0]);
            table.put("Positions", positions);
            table.put("SeesTarget", seesTarget);
            table.put("EstimatedRobotPose3d", estimatedRobotPose3d);
            table.put("EstimatedRobotPoseTimestampSeconds", estimatedRobotPose3dTimestampSeconds);
            // table.put("EstimatedRobotPoseTargets", estimatedRobotPose3dTargets);
            table.put("EstimatedRobotPose3dTargets", estimatedRobotPose3dTargets);
        }

        @Override
        public void fromLog(LogTable table) {
            yaw = table.get("Yaw", yaw);
            roll = table.get("Roll", roll);
            latencies = table.get("Latencies", latencies);
            results[0] = table.get("Results/1", results[0]);
            positions = table.get("Positions", positions);
            seesTarget = table.get("SeesTarget", seesTarget);
            estimatedRobotPose3d = table.get("EstimatedRobotPose3d", estimatedRobotPose3d);
            estimatedRobotPose3dTimestampSeconds = table.get("EstimatedRobotPoseTimestampSeconds",
                estimatedRobotPose3dTimestampSeconds);
            // estimatedRobotPose3dTargets =
            // table.get("EstimatedRobotPoseTargets", estimatedRobotPose3dTargets);
            estimatedRobotPose3dTargets =
                table.get("EstimatedRobotPose3dTargets", estimatedRobotPose3dTargets);
        }

        public SwerveInputsAutoLogged clone() {
            SwerveInputsAutoLogged copy = new SwerveInputsAutoLogged();
            copy.yaw = this.yaw;
            copy.roll = this.roll;
            copy.latencies = this.latencies.clone();
            copy.results = this.results.clone();
            copy.positions = this.positions.clone();
            copy.seesTarget = this.seesTarget.clone();
            copy.estimatedRobotPose3d = this.estimatedRobotPose3d.clone();
            copy.estimatedRobotPose3dTimestampSeconds =
                this.estimatedRobotPose3dTimestampSeconds.clone();
            copy.estimatedRobotPose3dTargets = this.estimatedRobotPose3dTargets.clone();
            return copy;
        }

        public float yaw;
        public float roll;
        public double[] latencies;
        public PhotonPipelineResult[] results;
        public Pose3d[] positions;
        public boolean[] seesTarget;
        public Pose3d[] estimatedRobotPose3d;
        public double[] estimatedRobotPose3dTimestampSeconds;
        public PhotonTrackedTarget[] estimatedRobotPose3dTargets;
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
