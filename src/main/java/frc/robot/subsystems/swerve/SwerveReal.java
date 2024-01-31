package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.PhotonCameraWrapper;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;

/** Real Class for Swerve */
public class SwerveReal implements SwerveIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

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

    /** Real Swerve Initializer */
    public SwerveReal() {

    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yaw = gyro.getYaw();
        inputs.roll = gyro.getRoll();
        inputs.frontLeftCameraLatency = frontLeftCam.latency();
        inputs.frontRightCameraLatency = frontRightCam.latency();
        inputs.backLeftCameraLatency = backLeftCam.latency();
        inputs.backRightCameraLatency = backRightCam.latency();
        inputs.frontLeftPhotonResult = frontLeftCam.photonCamera.getLatestResult();
        inputs.frontRightPhotonResult = frontRightCam.photonCamera.getLatestResult();
        inputs.backLeftPhotonResult = backLeftCam.photonCamera.getLatestResult();
        inputs.backRightPhotonResult = backRightCam.photonCamera.getLatestResult();
    }

    @Override
    public SwerveModule createSwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleReal(moduleNumber, driveMotorID, angleMotorID, cancoderID,
                angleOffset));
    }

}
