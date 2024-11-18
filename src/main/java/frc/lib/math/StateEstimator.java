package frc.lib.math;

import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.vision.CameraConstants;

/**
 * Primary interface for estimating robot drivetrain state.
 */
public interface StateEstimator {

    public void init(SwerveDriveKinematics kinematics, Rotation2d gyroYaw,
        SwerveModulePosition[] positions, Pose2d initialPose);

    public void resetPosition(Rotation2d gyroYaw, SwerveModulePosition[] positions,
        Pose2d initialPose);

    public Pose2d getPoseEstimate();

    public void swerveOdometryUpdate(Rotation2d gyro, SwerveModulePosition[] positions);

    public void visionUpdate(PhotonPipelineResult data, CameraConstants constants);

    public void collisionUpdate();

}
