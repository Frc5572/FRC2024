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
            for (int i = 0; i < 4; i++) {
                table.put("Latencies/" + i, latencies[i]);
                table.put("Results/" + i, PhotonPipelineResult.proto, results[i]);
                table.put("Positions/" + i, positions[i]);
                table.put("SeesTarget/" + i, seesTarget[i]);
                table.put("EstimatedRobotPose3d/" + i, estimatedRobotPose3d[i]);
                table.put("EstimatedRobotPoseTimestampSeconds/" + i,
                    estimatedRobotPose3dTimestampSeconds[i]);
                // table.put("EstimatedRobotPoseTargets", estimatedRobotPose3dTargets);
                table.put("EstimatedRobotPose3dTargets/" + i, estimatedRobotPose3dTargets[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            yaw = table.get("Yaw", yaw);
            roll = table.get("Roll", roll);
            for (int i = 0; i < 4; i++) {
                latencies[i] = table.get("Latencies/" + i, latencies[i]);
                results[i] = table.get("Results/" + i, results[i]);
                positions[i] = table.get("Positions/" + i, positions[i]);
                seesTarget[i] = table.get("SeesTarget/" + i, seesTarget[i]);
                estimatedRobotPose3d[i] =
                    table.get("EstimatedRobotPose3d/" + i, estimatedRobotPose3d[i]);
                estimatedRobotPose3dTimestampSeconds[i] =
                    table.get("EstimatedRobotPoseTimestampSeconds/" + i,
                        estimatedRobotPose3dTimestampSeconds[i]);
                // estimatedRobotPose3dTargets =
                // table.get("EstimatedRobotPoseTargets", estimatedRobotPose3dTargets);
                estimatedRobotPose3dTargets[i] =
                    table.get("EstimatedRobotPose3dTargets/" + i, estimatedRobotPose3dTargets[i]);
            }
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
        public boolean[] latencies;
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
