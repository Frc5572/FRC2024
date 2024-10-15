package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FieldConstants;
import frc.lib.util.photon.LoggedPhotonCamera;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;

/** PhotonVision subsystem. */
public class Vision extends SubsystemBase {

    private final LoggedPhotonCamera[] cameras;
    private static final AprilTagFieldLayout layout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private static final double fieldBorderMargin = 0.5;
    private static final double zMargin = 0.75;

    /** Create new vision system. */
    public Vision(boolean isSim) {
        super("Vision");
        cameras = new LoggedPhotonCamera[] {
            new LoggedPhotonCamera(Constants.CameraConstants.FrontRightFacingCamera.CAMERA_NAME,
                Constants.CameraConstants.FrontRightFacingCamera.CAMERA_IP,
                Constants.CameraConstants.FrontRightFacingCamera.KCAMERA_TO_ROBOT,
                Constants.CameraConstants.FrontRightFacingCamera.STDDEV_FACTOR)};
    }

    @Override
    public void periodic() {
        for (var cam : cameras) {
            cam.periodic();
        }
        List<VisionObservation> visionObservations = new ArrayList<>();
        for (int idx = 0; idx < cameras.length; idx++) {
            var result = cameras[idx].getLatestResult();
            Pose3d cameraPose = null;
            Pose3d robotPose3d = null;
            boolean useVisionRotation = false;
            if (result.targets.size() == 1) {
                double ambiguity = result.getBestTarget().getPoseAmbiguity();
                if (ambiguity < 0.4) {
                    cameraPose = new Pose3d().plus(result.getBestTarget().getBestCameraToTarget())
                        .relativeTo(layout.getOrigin());
                }
            } else if (result.targets.size() > 1) {
                var best_tf = result.getMultiTagResult().estimatedPose.best;
                cameraPose = new Pose3d().plus(best_tf).relativeTo(layout.getOrigin());
                useVisionRotation = true;
            }
            if (cameraPose == null) {
                continue;
            }
            robotPose3d = cameraPose.plus(cameras[idx].cameraToRobot);
            // exit if robot pose is off the field
            if (robotPose3d.getX() < -fieldBorderMargin
                || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
                || robotPose3d.getY() < -fieldBorderMargin
                || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
                || robotPose3d.getZ() < -zMargin || robotPose3d.getZ() > zMargin) {
                continue;
            }
            Pose2d robotPose = robotPose3d.toPose2d();
            List<Pose3d> tagPoses =
                result.targets.stream().map((x) -> layout.getTagPose(x.getFiducialId()).get())
                    .filter(x -> x != null).toList();
            if (tagPoses.isEmpty()) {
                continue;
            }

            // Calculate average distance to tag
            double totalDistance = 0.0;
            for (Pose3d tagPose : tagPoses) {
                totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
            }
            double avgDistance = totalDistance / tagPoses.size();

            double xyStdDev = Constants.CameraConstants.XY_STD_DEV_COEFF
                * Math.pow(avgDistance, 2.0) / tagPoses.size() * cameras[idx].stdDevFactor;
            double thetaStdDev = useVisionRotation
                ? Constants.CameraConstants.THETA_STD_DEV_COEFF * Math.pow(avgDistance, 2.0)
                    / tagPoses.size() * cameras[idx].stdDevFactor
                : Double.POSITIVE_INFINITY;
            visionObservations.add(new VisionObservation(robotPose, result.getTimestampSeconds(),
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        }
        visionObservations.stream().sorted(Comparator.comparingDouble(VisionObservation::timestamp))
            .forEach(RobotState.getInstance()::addVisionObservation);
    }

}
