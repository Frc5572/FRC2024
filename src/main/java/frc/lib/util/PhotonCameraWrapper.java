package frc.lib.util;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * PhotonCamera-based Pose Estimator.
 */
public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    /**
     * PhotonCamera-based Pose Estimator.
     *
     * @param cameraName camera name (not to be confused with mDNS name).
     * @param robotToCam transform from robot body coordinates to camera coordinates.
     */
    public PhotonCameraWrapper(String cameraName, Transform3d robotToCam) {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        photonCamera = new PhotonCamera(cameraName);

        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the
        // field.
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);

    }

    /**
     * Gets if photonvision can see a target.
     */
    public boolean seesTarget() {
        return photonCamera.getLatestResult().hasTargets();
    }

    /**
     * Get estimated pose without a prior.
     *
     * @return an estimated Pose2d based solely on apriltags
     */
    public Optional<Pose2d> getInitialPose() {
        var res = photonCamera.getLatestResult();
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
        var res = photonCamera.getLatestResult();
        return Timer.getFPGATimestamp() - res.getTimestampSeconds();
    }

    /**
     * @param prevEstimatedRobotPose The current best guess at robot pose
     *
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
     *         create the estimate
     */
    public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        var res = photonCamera.getLatestResult();
        SmartDashboard.putNumber("photonLatency",
            Timer.getFPGATimestamp() - res.getTimestampSeconds());
        if (Timer.getFPGATimestamp() - res.getTimestampSeconds() > 0.4) {
            return null;
        }
        if (photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return null;
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update().get();
    }
}
