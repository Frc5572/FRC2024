package frc.robot.subsystems.swerve.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Gyro implementation for the Canandgyro Boron */
public class SwerveBoron implements SwerveIO {

    /** The gyroscope as a raw object. */
    public Canandgyro gyro = new Canandgyro(Constants.Swerve.gyroId);

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yawPosition = gyro.getRotation2d();
        inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityYaw());
        inputs.connected = true;
    }

    @Override
    public void overridePose(Pose2d pose) {}

}
