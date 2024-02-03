package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

/**
 * Class for ShooterVortex
 */
public class ShooterVortex implements ShooterIO {
    public final CANSparkFlex shooterMotor =
        new CANSparkFlex(Constants.Motors.Shooter.SHOOTER_TOP_ID, MotorType.kBrushless);
    private RelativeEncoder shooterEncoder;

    public ShooterVortex() {
        shooterEncoder = shooterMotor.getEncoder();
    }

    public void setMotor(double power) {
        shooterMotor.setVoltage(power);
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.shooterVelocityRotPerMin = shooterEncoder.getVelocity();
        inputs.shooterSupplyVoltage = shooterMotor.getBusVoltage();
        inputs.shooterAmps = shooterMotor.getOutputCurrent();
    }
}
