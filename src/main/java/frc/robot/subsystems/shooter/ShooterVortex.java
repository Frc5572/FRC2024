package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

/**
 * Class for ShooterVortex
 */
public class ShooterVortex implements ShooterIO {
    public final CANSparkFlex topShooterMotor =
        new CANSparkFlex(Constants.Motors.Shooter.SHOOTER_TOP_ID, MotorType.kBrushless);
    public final CANSparkFlex bottomShooterMotor =
        new CANSparkFlex(Constants.Motors.Shooter.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
    private RelativeEncoder topShooterEncoder;
    private RelativeEncoder bottomShooterEncoder;

    public ShooterVortex() {
        topShooterEncoder = topShooterMotor.getEncoder();
        bottomShooterEncoder = bottomShooterMotor.getEncoder();
    }

    public void setTopMotor(double power) {
        topShooterMotor.setVoltage(power);
    }

    public void setBottomMotor(double power) {
        bottomShooterMotor.setVoltage(power);
    }


    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.topShooterVelocityRotPerMin = topShooterEncoder.getVelocity();
        inputs.bottomShooterVelocityRotPerMin = bottomShooterEncoder.getVelocity();
        inputs.topShooterSupplyVoltage = topShooterMotor.getBusVoltage();
        inputs.bottomShooterSupplyVoltage = topShooterMotor.getBusVoltage();
        inputs.topShooterAmps = topShooterMotor.getOutputCurrent();
        inputs.bottomShooterAmps = topShooterMotor.getOutputCurrent();

    }
}
