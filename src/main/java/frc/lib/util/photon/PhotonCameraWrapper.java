package frc.lib.util.photon;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.photon.PhotonIO.PhotonInputs;

/**
 * PhotonCamera-based Pose Estimator.
 */
public class PhotonCameraWrapper {
    public PhotonIO io;
    public PhotonInputs inputs = new PhotonInputs();
    public PhotonIOPoseEstimator photonPoseEstimator;

    /**
     * PhotonCamera-based Pose Estimator.
     *
     * @param io Camera IO.
     * @param robotToCam transform from robot body coordinates to camera coordinates.
     */
    public PhotonCameraWrapper(PhotonIO io, Transform3d robotToCam) {
        this.io = io;

        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the
        // field.
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // Create pose estimator
        photonPoseEstimator = new PhotonIOPoseEstimator(fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.inputs, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
    }

    /**
     * Update inputs and such.
     */
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("PhotonVision/" + inputs.name, inputs);
    }

    /**
     * Gets if photonvision can see a target.
     */
    public boolean seesTarget() {
        return inputs.result.hasTargets();
    }

    /**
     * Get estimated pose without a prior.
     *
     * @return an estimated Pose2d based solely on apriltags
     */
    public Optional<Pose2d> getInitialPose() {
        var res = inputs.result;
        SmartDashboard.putNumber("Heartbeat", inputs.timeSinceLastHeartbeat);
        if (inputs.timeSinceLastHeartbeat > 0.5) {
            return Optional.empty();
        }
        SmartDashboard.putNumber("lastTimePhton", res.getTimestampSeconds());
        if (res.hasTargets()) {
            var target = res.getBestTarget();
            var camToTargetTrans = target.getBestCameraToTarget();
            var aprilTagPose =
                photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (aprilTagPose.isPresent()) {
                var camPose = aprilTagPose.get().transformBy(camToTargetTrans.inverse());
                var robotPose =
                    camPose.transformBy(photonPoseEstimator.getRobotToCameraTransform()).toPose2d();
                return Optional.of(robotPose);
            }
        }
        return Optional.empty();
    }

    public double latency() {
        var res = inputs.result;
        return Timer.getFPGATimestamp() - res.getTimestampSeconds();
    }

    /**
     * @param prevEstimatedRobotPose The current best guess at robot pose
     *
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
     *         create the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        var res = inputs.result;
        // SmartDashboard.putNumber("photonLatency",
        // Timer.getFPGATimestamp() - res.getTimestampSeconds());
        if (Timer.getFPGATimestamp() - res.getTimestampSeconds() > 0.4) {
            return Optional.empty();
        }
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}
