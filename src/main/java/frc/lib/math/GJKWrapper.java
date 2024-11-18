package frc.lib.math;

import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.vision.CameraConstants;

public class GJKWrapper implements StateEstimator {

    private final StateEstimator inner;

    public GJKWrapper(StateEstimator inner) {
        this.inner = inner;
    }

    @Override
    public void init(SwerveDriveKinematics kinematics, Rotation2d gyroYaw,
        SwerveModulePosition[] positions, Pose2d initialPose) {
        inner.init(kinematics, gyroYaw, positions, initialPose);
    }

    @Override
    public void resetPosition(Rotation2d gyroYaw, SwerveModulePosition[] positions,
        Pose2d initialPose) {
        inner.resetPosition(gyroYaw, positions, initialPose);
    }

    @Override
    public Pose2d getPoseEstimate() {
        return inner.getPoseEstimate();
    }

    @Override
    public void swerveOdometryUpdate(Rotation2d gyro, SwerveModulePosition[] positions) {
        inner.swerveOdometryUpdate(gyro, positions);
    }

    @Override
    public void visionUpdate(PhotonPipelineResult data, CameraConstants constants) {
        inner.visionUpdate(data, constants);
    }

    @Override
    public void collisionUpdate() {
        // TODO cacheing GJK solve using field geometry
        // https://github.com/JuliaRobotics/EnhancedGJK.jl
    }

}
