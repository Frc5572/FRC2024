package frc.robot.subsystems.swerve.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveBoron implements SwerveIO {

    public Canandgyro gyro = new Canandgyro(Constants.Swerve.config.gyroId);

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yawPosition = gyro.getRotation2d();
        inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityYaw());
        inputs.connected = true;
    }

}
