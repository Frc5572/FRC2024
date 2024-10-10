package frc.lib.util.photon;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * PhotonCamera-based Pose Estimator.
 */
public class PhotonCameraWrapper {
    public LoggedPhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;

    /**
     * PhotonCamera-based Pose Estimator.
     *
     * @param robotToCam transform from robot body coordinates to camera coordinates.
     */
    public PhotonCameraWrapper(String name, String cameraIP, Transform3d robotToCam) {
        this.camera = new LoggedPhotonCamera(name, cameraIP);

        // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the
        // field.
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    /**
     * Update inputs and such.
     */
    public void periodic() {
        this.camera.periodic();
    }

    /**
     * Gets if photonvision can see a target.
     */
    public boolean seesTarget() {
        var result = this.camera.getLatestResult();
        return result != null && result.hasTargets();
    }

    /** A PhotonVision tag solve. */
    public static class VisionObservation {
        public int fudicialId;
        public Pose2d robotPose;
        public Matrix<N3, N1> stdDev;

        /** All fields constructor. */
        public VisionObservation(int fudicialId, Pose2d robotPose, Matrix<N3, N1> stdDev) {
            this.fudicialId = fudicialId;
            this.robotPose = robotPose;
            this.stdDev = stdDev;
        }
    }

    /**
     * Get estimated pose without a prior.
     *
     * @return an estimated Pose2d based solely on apriltags
     */
    public Optional<VisionObservation> getInitialPose() {
        var res = this.camera.getLatestResult();
        if (res == null || Timer.getFPGATimestamp() - res.getTimestampSeconds() > 0.5) {
            SmartDashboard.putString("reason", "res is null or heartbeat too long");
            return Optional.empty();
        }
        if (res.hasTargets()) {
            var target = res.getBestTarget();
            var camToTargetTrans = target.getBestCameraToTarget();
            var aprilTagPose =
                photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (aprilTagPose.isPresent()) {
                var camPose = aprilTagPose.get().transformBy(camToTargetTrans.inverse());
                var robotPose =
                    camPose.transformBy(photonPoseEstimator.getRobotToCameraTransform()).toPose2d();

                Translation2d toTarget =
                    new Pose3d().plus(camToTargetTrans).toPose2d().getTranslation();
                double stdDev =
                    toTarget.getX() * toTarget.getX() + toTarget.getY() * toTarget.getY();
                return Optional.of(new VisionObservation(target.getFiducialId(), robotPose,
                    VecBuilder.fill(stdDev * Constants.CameraConstants.XY_STD_DEV_COEFF,
                        stdDev * Constants.CameraConstants.XY_STD_DEV_COEFF,
                        stdDev * Constants.CameraConstants.THETA_STD_DEV_COEFF)));
            }
        }
        SmartDashboard.putString("reason", "no targets");
        return Optional.empty();
    }

    public double latency() {
        var res = this.camera.getLatestResult();
        return Timer.getFPGATimestamp() - res.getTimestampSeconds();
    }

    /**
     * @param prevEstimatedRobotPose The current best guess at robot pose
     *
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
     *         create the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        var res = this.camera.getLatestResult();

        // photonPoseEstimator.setRobotToCameraTransform(new Transform3d(
        // new Translation3d(Units.inchesToMeters(SmartDashboard.getNumber("tx", 0)),
        // Units.inchesToMeters(SmartDashboard.getNumber("ty", 0)),
        // Units.inchesToMeters(SmartDashboard.getNumber("tz", 0))),
        // new Rotation3d(Math.toRadians(SmartDashboard.getNumber("rx", 0)),
        // Math.toRadians(SmartDashboard.getNumber("ry", 0)),
        // Math.toRadians(SmartDashboard.getNumber("rz", 0)))));

        // SmartDashboard.putNumber("photonLatency",
        // Timer.getFPGATimestamp() - res.getTimestampSeconds());
        if (Timer.getFPGATimestamp() - res.getTimestampSeconds() > 0.4) {
            SmartDashboard.putString("reason", "too long");
            return Optional.empty();
        }
        if (photonPoseEstimator == null) {
            SmartDashboard.putString("reason", "no estimator");
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        SmartDashboard.putString("reason", "good");
        return photonPoseEstimator.update();
    }
}
