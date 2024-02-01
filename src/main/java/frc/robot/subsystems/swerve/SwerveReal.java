package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.PhotonCameraWrapper;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;

/** Real Class for Swerve */
public class SwerveReal implements SwerveIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

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
        for (int i = 0; i < cameras.length; i++) {
            inputs.latencies[i] = cameras[i].latency();
            inputs.positions[i] =
                cameras[i].getEstimatedGlobalPose(previousPose).map((x) -> x.estimatedPose);
            inputs.results[i] = cameras[i].photonCamera.getLatestResult();
            inputs.seesTarget[i] = cameras[i].seesTarget();
        }

    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(int i,
        Pose2d prevEstimatedRobotPose) {
        return this.cameras[i].getEstimatedGlobalPose(prevEstimatedRobotPose);
    }

    // /**
    // * @param prevEstimatedRobotPose The current best guess at robot pose
    // *
    // * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
    // * create the estimate
    // */
    // @Override
    // private Optional<EstimatedRobotPose> getFrontLeftEstimatedGlobalPose(
    // Pose2d prevEstimatedRobotPose, PhotonPipelineResult result) {
    // var res = frontLeftCam.photonCamera.getLatestResult();
    // if (Timer.getFPGATimestamp() - res.getTimestampSeconds() > 0.4) {
    // return Optional.empty();
    // }

    // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    // return photonPoseEstimator.update();
    // }

    @Override
    public Optional<EstimatedRobotPose> getFrontLeftEstimatedGlobalPose() {
        var res = frontLeftCam.photonCamera.getLatestResult();
        if (Timer.getFPGATimestamp() - res.getTimestampSeconds() > 0.4) {
            return Optional.empty();
        }

        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    @Override
    public SwerveModule createSwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleReal(moduleNumber, driveMotorID, angleMotorID, cancoderID,
                angleOffset));
    }

}
