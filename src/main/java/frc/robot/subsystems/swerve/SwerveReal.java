package frc.robot.subsystems.swerve;

import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
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
    private PhotonCameraWrapper[] cameras =
        {new PhotonCameraWrapper(Constants.CameraConstants.BackRightFacingCamera.CAMERA_NAME,
            Constants.CameraConstants.BackRightFacingCamera.KCAMERA_TO_ROBOT.inverse())};

    /** Real Swerve Initializer */
    public SwerveReal() {}

    @Override
    public void updateInputs(SwerveInputs inputs, Pose2d previousPose) {
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.latencies = new boolean[cameras.length];
        inputs.positions = new Pose3d[cameras.length];
        inputs.results = new PhotonPipelineResult[cameras.length];
        inputs.seesTarget = new boolean[cameras.length];
        inputs.estimatedRobotPose3d = new Pose3d[cameras.length];
        inputs.estimatedRobotPose3dTimestampSeconds = new double[cameras.length];
        inputs.estimatedRobotPose3dTargets = new PhotonTrackedTarget[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            if (cameras[i].getEstimatedGlobalPose(previousPose) != null) {
                inputs.latencies[i] = cameras[i].latency() < 0.6;
                inputs.results[i] = cameras[i].photonCamera.getLatestResult();
                inputs.seesTarget[i] = cameras[i].seesTarget();
                inputs.positions[i] = cameras[i].getEstimatedGlobalPose(previousPose).estimatedPose;
                inputs.estimatedRobotPose3d[i] =
                    cameras[i].getEstimatedGlobalPose(previousPose).estimatedPose;
                inputs.estimatedRobotPose3dTimestampSeconds[i] =
                    cameras[i].getEstimatedGlobalPose(previousPose).timestampSeconds;
                inputs.estimatedRobotPose3dTargets[i] =
                    cameras[i].getEstimatedGlobalPose(previousPose).targetsUsed.get(0);
            } else {
                inputs.results[i] = new PhotonPipelineResult();
                inputs.positions[i] = new Pose3d();
                inputs.estimatedRobotPose3d[i] = new Pose3d();
                inputs.estimatedRobotPose3dTimestampSeconds[i] = 0.0;
                List<TargetCorner> targetCornersList =
                    List.of(new TargetCorner(0.0, 0.0), new TargetCorner(0.0, 0.0),
                        new TargetCorner(0.0, 0.0), new TargetCorner(0.0, 0.0));
                inputs.estimatedRobotPose3dTargets[i] = new PhotonTrackedTarget(0, 0, 0, 0, 500,
                    null, null, i, targetCornersList, targetCornersList);
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
