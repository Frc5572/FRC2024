package frc.robot.subsystems.swerve;

import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
        inputs.numCameras = cameras.length;
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.latencies = new boolean[cameras.length];
        inputs.results = new PhotonPipelineResult[cameras.length];
        inputs.seesTarget = new boolean[cameras.length];
        inputs.estimatedRobotPose3d = new Pose3d[cameras.length];
        inputs.estimatedRobotPose3dTimestampSeconds = new double[cameras.length];
        inputs.estimatedRobotPose3dTargets = new PhotonTrackedTarget[cameras.length];
        for (int i = 0; i < inputs.numCameras; i++) {
            if (cameras[i].getEstimatedGlobalPose(previousPose).isPresent()) {
                EstimatedRobotPose robotPose =
                    cameras[i].getEstimatedGlobalPose(previousPose).get();
                inputs.latencies[i] = cameras[i].latency() < 0.6;
                inputs.results[i] = cameras[i].photonCamera.getLatestResult();
                inputs.seesTarget[i] = cameras[i].seesTarget();
                inputs.estimatedRobotPose3d[i] = robotPose.estimatedPose;
                inputs.estimatedRobotPose3dTimestampSeconds[i] = robotPose.timestampSeconds;
                inputs.estimatedRobotPose3dTargets[i] = robotPose.targetsUsed.get(0);
            } else {
                inputs.results[i] = new PhotonPipelineResult();
                inputs.estimatedRobotPose3d[i] = new Pose3d();
                inputs.estimatedRobotPose3dTimestampSeconds[i] = 0.0;
                List<TargetCorner> targetCornersList =
                    List.of(new TargetCorner(0.0, 0.0), new TargetCorner(0.0, 0.0),
                        new TargetCorner(0.0, 0.0), new TargetCorner(0.0, 0.0));
                inputs.estimatedRobotPose3dTargets[i] = new PhotonTrackedTarget(0, 0, 0, 0, 500,
                    new Transform3d(), new Transform3d(), 0, targetCornersList, targetCornersList);
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
