package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.PhotonCameraWrapper;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;
import frc.robot.Constants;

/** IO Class for Swerve */
public interface SwerveIO {

    public PhotonCameraWrapper frontLeftCam =
        new PhotonCameraWrapper(Constants.CameraConstants.FrontLeftFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.FrontLeftFacingCamera.KCAMERA_TO_ROBOT.inverse());
    public PhotonCameraWrapper frontRightCam =
        new PhotonCameraWrapper(Constants.CameraConstants.FrontRightFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.FrontRightFacingCamera.KCAMERA_TO_ROBOT.inverse());
    public PhotonCameraWrapper backLeftCam =
        new PhotonCameraWrapper(Constants.CameraConstants.BackLeftFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.BackLeftFacingCamera.KCAMERA_TO_ROBOT.inverse());
    public PhotonCameraWrapper backRightCam =
        new PhotonCameraWrapper(Constants.CameraConstants.BackRightFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.BackRightFacingCamera.KCAMERA_TO_ROBOT.inverse());

    /** Inputs Class for Swerve */
    @AutoLog
    public static class SwerveInputs {
        public float yaw;
        public float roll;
        public double frontLeftCameraLatency;
        public double frontRightCameraLatency;
        public double backLeftCameraLatency;
        public double backRightCameraLatency;
        public PhotonPipelineResult frontLeftPhotonResult;
        public PhotonPipelineResult frontRightPhotonResult;
        public PhotonPipelineResult backLeftPhotonResult;
        public PhotonPipelineResult backRightPhotonResult;

    }

    public default void updateInputs(SwerveInputs inputs) {}

    /** Instantiating SwerveModules */
    public default SwerveModule createSwerveModule(int moduleNumber, int driveMotorID,
        int angleMotorID, int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleIO() {});
    }

}
