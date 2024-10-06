package frc.robot;

import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.CameraConstants.BackLeftFacingCamera;
import frc.robot.Constants.CameraConstants.FrontLeftFacingCamera;
import frc.robot.Constants.CameraConstants.FrontRightFacingCamera;
import frc.robot.subsystems.vision.PhotonCameraProperties;

public class VisionConstants {
    public static final String APRIL_TAGS_VISION_PATH = "Vision/AprilTags/";
    public static final AprilTagFieldLayout fieldLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public static final int MINIMUM_TAGS_NUM = 2;
    /* default standard error for vision observation, if only one apriltag observed */
    public static final double TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION = 0.3,
        ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION = Math.toRadians(10),

        // only do odometry calibration if translational standard error if it is not greater than
        TRANSLATIONAL_STANDARD_ERROR_THRESHOLD = 0.5,
        // only do gyro calibration if rotational standard error is very, very small
        ROTATIONAL_STANDARD_ERROR_THRESHOLD = Math.toRadians(5),
        ROTATIONAL_ERROR_WITH_GYRO_DISCARD_RESULT = Math.toRadians(15),

        ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS = 0.02,
        // we trust the IMU very much (recommend 0.1 for Pigeon2, 0.5 for NavX)
        GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS = Math.toRadians(0.1);

    private static final PhotonCameraProperties frontLeftCam = new PhotonCameraProperties(
        FrontLeftFacingCamera.CAMERA_NAME, FrontLeftFacingCamera.CAMERA_IP, 30.0, 18.0, 5.0,
        Rotation2d.fromDegrees(68), 0.6, 0.2, 1280, 800, FrontLeftFacingCamera.KCAMERA_TO_ROBOT);
    private static final PhotonCameraProperties frontRightCam = new PhotonCameraProperties(
        FrontRightFacingCamera.CAMERA_NAME, FrontRightFacingCamera.CAMERA_IP, 30.0, 18.0, 5.0,
        Rotation2d.fromDegrees(68), 0.6, 0.2, 1600, 1200, FrontRightFacingCamera.KCAMERA_TO_ROBOT);
    private static final PhotonCameraProperties backLeftCam = new PhotonCameraProperties(
        BackLeftFacingCamera.CAMERA_NAME, BackLeftFacingCamera.CAMERA_IP, 30.0, 18.0, 5.0,
        Rotation2d.fromDegrees(68), 0.6, 0.2, 1600, 1200, BackLeftFacingCamera.KCAMERA_TO_ROBOT);
    public static final List<PhotonCameraProperties> photonVisionCameras =
        List.of(frontLeftCam, frontRightCam, backLeftCam);
    // public static final List<PhotonCameraProperties> photonVisionCameras =
    // List.of(new PhotonCameraProperties("FrontCam", 30, 18, 5, 68, 0.6, 0.2, 1280, 720,
    // new Translation2d(0.2, 0), // the outing position of the camera in relative to the robot
    // // center
    // 0.3, // the mounting height, in meters
    // Rotation2d.fromDegrees(0), // the camera facing, 0 is front, positive is
    // // counter-clockwise
    // 24, // camera pitch angle, in degrees
    // 180 // camera roll angle, 0 for up-right and 180 for upside-down
    // ), new PhotonCameraProperties("FrontLeftCam", 30, 18, 5, 68, 0.6, 0.2, 1280, 720,
    // new Translation2d(0.2, 0.15), 0.3, Rotation2d.fromDegrees(35), 30, 180 // upside-down
    // ), new PhotonCameraProperties("FrontRightCam", 30, 18, 5, 68, 0.6, 0.2, 1280, 720,
    // new Translation2d(0.2, -0.15), 0.3, Rotation2d.fromDegrees(-35), 30, 180 // upside-down
    // ), new PhotonCameraProperties("BackLeftCam", 30, 18, 5, 68, 0.6, 0.2, 1280, 720,
    // new Translation2d(-0.2, 0.15), 0.3, Rotation2d.fromDegrees(60), 30, 180 // upside-down
    // ), new PhotonCameraProperties("BackRightCam", 30, 18, 5, 68, 0.6, 0.2, 1280, 720,
    // new Translation2d(-0.2, -0.15), 0.3, Rotation2d.fromDegrees(-60), 30, 180 // upside-down
    // ));
}
