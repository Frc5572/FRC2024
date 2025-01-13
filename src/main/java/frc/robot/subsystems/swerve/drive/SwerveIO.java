package frc.robot.subsystems.swerve.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** IO for Swerve */
public interface SwerveIO {

    /** Inputs for swerve drive excluding module inputs. */
    @AutoLog
    public static class SwerveInputs {
        public boolean connected;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
    }

    /** Update the inputs */
    public void updateInputs(SwerveInputs inputs);

    /** Override actual pose. Used only in sim. */
    public void overridePose(Pose2d pose);

}
