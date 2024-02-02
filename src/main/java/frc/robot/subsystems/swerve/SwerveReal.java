package frc.robot.subsystems.swerve;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
            Constants.CameraConstants.FrontLeftFacingCamera.KCAMERA_TO_ROBOT.inverse()),
        new PhotonCameraWrapper(Constants.CameraConstants.FrontRightFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.FrontRightFacingCamera.KCAMERA_TO_ROBOT.inverse()),
        new PhotonCameraWrapper(Constants.CameraConstants.BackLeftFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.BackLeftFacingCamera.KCAMERA_TO_ROBOT.inverse()),
        new PhotonCameraWrapper(Constants.CameraConstants.BackRightFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.BackRightFacingCamera.KCAMERA_TO_ROBOT.inverse())};

    /** Real Swerve Initializer */
    public SwerveReal() {}

    @Override
    public void updateInputs(SwerveInputs inputs, Pose2d previousPose) {
        inputs.yaw = gyro.getYaw();
        inputs.roll = gyro.getRoll();
        inputs.latencies = new double[4];
        inputs.positions = new Pose3d[4];
        inputs.results = new PhotonPipelineResult[4];
        inputs.seesTarget = new boolean[4];
        inputs.estimatedRobotPose3d = new Pose3d[4];
        inputs.estimatedRobotPose3dTimestampSeconds = new double[4];
        inputs.estimatedRobotPose3dTargets = new PhotonTrackedTarget[4];
        for (int i = 0; i < cameras.length; i++) {
            inputs.latencies[i] = cameras[i].latency();
            inputs.positions[i] = cameras[i].getEstimatedGlobalPose(previousPose).estimatedPose;
            inputs.results[i] = cameras[i].photonCamera.getLatestResult();
            inputs.seesTarget[i] = cameras[i].seesTarget();
            inputs.estimatedRobotPose3d[i] =
                cameras[i].getEstimatedGlobalPose(previousPose).estimatedPose;
            inputs.estimatedRobotPose3dTimestampSeconds[i] =
                cameras[i].getEstimatedGlobalPose(previousPose).timestampSeconds;
            inputs.estimatedRobotPose3dTargets[i] =
                cameras[i].getEstimatedGlobalPose(previousPose).targetsUsed.get(0);
        }

    }

    public EstimatedRobotPose getEstimatedGlobalPose(int i, Pose2d prevEstimatedRobotPose) {
        return this.cameras[i].getEstimatedGlobalPose(prevEstimatedRobotPose);
    }

    @Override
    public SwerveModule createSwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleReal(moduleNumber, driveMotorID, angleMotorID, cancoderID,
                angleOffset));
    }

}
