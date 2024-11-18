package frc.lib.math;

import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.vision.CameraConstants;

/** State estimator using WPILib stuff */
public class StandardStateEstimator implements StateEstimator {

    private Pose2d currentPose = new Pose2d();
    private SwerveDrivePoseEstimator estimator = null;

    @Override
    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    @Override
    public void swerveOdometryUpdate(Rotation2d gyro, SwerveModulePosition[] positions) {
        estimator.update(gyro, positions);
    }

    @Override
    public void init(SwerveDriveKinematics kinematics, Rotation2d gyroYaw,
        SwerveModulePosition[] positions, Pose2d initialPose) {
        estimator = new SwerveDrivePoseEstimator(kinematics, gyroYaw, positions, initialPose);
    }

    @Override
    public void resetPosition(Rotation2d gyroYaw, SwerveModulePosition[] positions,
        Pose2d initialPose) {
        estimator.resetPosition(gyroYaw, positions, initialPose);
    }

    @Override
    public void collisionUpdate() {
        // WPILib stuff doesn't handle field collisions.
    }

    @Override
    public void visionUpdate(PhotonPipelineResult data, CameraConstants constants) {
        // TODO vision stuff
    }

}
