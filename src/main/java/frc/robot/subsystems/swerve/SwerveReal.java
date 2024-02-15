package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.PhotonCameraWrapper;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;

/** Real Class for Swerve */
public class SwerveReal implements SwerveIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    /*
     * Camera Order: 0 - Front Left 1 - Front RIght 2 - Back Left 3 - Back Right
     */
    private PhotonCameraWrapper[] cameras = {
        new PhotonCameraWrapper(Constants.CameraConstants.FrontLeftFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.FrontLeftFacingCamera.KCAMERA_TO_ROBOT),
        new PhotonCameraWrapper(Constants.CameraConstants.FrontRightFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.FrontRightFacingCamera.KCAMERA_TO_ROBOT),
        new PhotonCameraWrapper(Constants.CameraConstants.BackLeftFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.BackLeftFacingCamera.KCAMERA_TO_ROBOT),
        new PhotonCameraWrapper(Constants.CameraConstants.BackRightFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.BackRightFacingCamera.KCAMERA_TO_ROBOT)};

    /** Real Swerve Initializer */
    public SwerveReal() {}

    @Override
    public void updateInputs(SwerveInputs inputs, Pose2d previousPose) {
        inputs.numCameras = cameras.length;
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();

        for (int i = 0; i < cameras.length; i++) {
            inputs.seesTarget[i] = cameras[i].seesTarget();
            if (inputs.seesTarget[i]) {
                Optional<EstimatedRobotPose> optionalRobotPose =
                    cameras[i].getEstimatedGlobalPose(previousPose);
                if (optionalRobotPose.isPresent()) {
                    EstimatedRobotPose robotPose = optionalRobotPose.get();
                    inputs.estimatedRobotPose3d[i] = robotPose.estimatedPose;
                    inputs.estimatedRobotPose3dTimestampSeconds[i] = robotPose.timestampSeconds;
                    inputs.estimatedRobotPose3dTargets[i] = robotPose.targetsUsed.get(0);
                }
                inputs.latencies[i] = cameras[i].latency() < 0.6;
                inputs.results[i] = cameras[i].photonCamera.getLatestResult();
            }
        }

    }

    @Override
    public SwerveModule createSwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleReal(moduleNumber, driveMotorID, angleMotorID, cancoderID,
                angleOffset));
    }

}
