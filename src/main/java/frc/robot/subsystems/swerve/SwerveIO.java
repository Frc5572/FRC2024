package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;

/** IO Class for Swerve */
public interface SwerveIO {

    /** Inputs Class for Swerve */

    public static class SwerveInputs implements LoggableInputs, Cloneable {

        public int numCameras;
        public float yaw;
        public float roll;
        public float pitch;
        public boolean[] latencies;
        public PhotonPipelineResult[] results;
        public boolean[] seesTarget;
        public Pose3d[] estimatedRobotPose3d;
        public double[] estimatedRobotPose3dTimestampSeconds;
        public PhotonTrackedTarget[] estimatedRobotPose3dTargets;

        public SwerveInputs() {
            int numCameras = 4;
            this.latencies = new boolean[numCameras];
            this.results = new PhotonPipelineResult[numCameras];
            this.seesTarget = new boolean[numCameras];
            this.estimatedRobotPose3d = new Pose3d[numCameras];
            this.estimatedRobotPose3dTimestampSeconds = new double[numCameras];
            this.estimatedRobotPose3dTargets = new PhotonTrackedTarget[numCameras];
            for (int i = 0; i < numCameras; i++) {
                this.results[i] = new PhotonPipelineResult();
                this.estimatedRobotPose3d[i] = new Pose3d();
                this.estimatedRobotPose3dTimestampSeconds[i] = 0.0;
                List<TargetCorner> targetCornersList =
                    List.of(new TargetCorner(0.0, 0.0), new TargetCorner(0.0, 0.0),
                        new TargetCorner(0.0, 0.0), new TargetCorner(0.0, 0.0));
                this.estimatedRobotPose3dTargets[i] = new PhotonTrackedTarget(0, 0, 0, 0, 500,
                    new Transform3d(), new Transform3d(), 0, targetCornersList, targetCornersList);
            }
        }

        @Override
        public void toLog(LogTable table) {
            table.put("numCameras", numCameras);
            table.put("Yaw", yaw);
            table.put("Roll", roll);
            table.put("Pitch", pitch);
            for (int i = 0; i < numCameras; i++) {
                table.put("Latencies/" + i, latencies[i]);
                table.put("Results/" + i, PhotonPipelineResult.proto, results[i]);
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
            numCameras = table.get("numCameras", numCameras);
            yaw = table.get("Yaw", yaw);
            roll = table.get("Roll", roll);
            pitch = table.get("Roll", pitch);
            for (int i = 0; i < numCameras; i++) {
                latencies[i] = table.get("Latencies/" + i, latencies[i]);
                results[i] = table.get("Results/" + i, results[i]);
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
            copy.latencies = this.latencies.clone();
            copy.results = this.results.clone();
            copy.seesTarget = this.seesTarget.clone();
            copy.estimatedRobotPose3d = this.estimatedRobotPose3d.clone();
            copy.estimatedRobotPose3dTimestampSeconds =
                this.estimatedRobotPose3dTimestampSeconds.clone();
            copy.estimatedRobotPose3dTargets = this.estimatedRobotPose3dTargets.clone();
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
