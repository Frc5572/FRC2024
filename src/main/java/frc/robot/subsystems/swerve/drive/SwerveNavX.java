package frc.robot.subsystems.swerve.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Gyro implementation for the NavX2 */
public class SwerveNavX implements SwerveIO {

    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yawPosition = Rotation2d.fromDegrees(gyro.getYaw());
        inputs.connected = true;
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());
    }

    @Override
    public void overridePose(Pose2d pose) {}

}
