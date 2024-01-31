package frc.robot.subsystems.swerve;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleIO;

/** IO Class for Swerve */
public interface SwerveIO {

    /** Inputs Class for Swerve */
    @AutoLog
    public static class SwerveInputs {
        public float yaw;
        public float roll;
        public double frontLeftCameraLatency;
        public Optional<Pose2d> frontLeftCameraInitialPose;
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

    public default Optional<Pose2d> getInitialPose() {
        return null;
    }

}
