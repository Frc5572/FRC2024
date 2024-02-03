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
    private RelativeEncoder topEncoder = topShooterMotor.getEncoder();
    private RelativeEncoder bottomEncoder = bottomShooterMotor.getEncoder();

    public ShooterVortex() {}

    public void setTopMotor(double power) {
        topShooterMotor.setVoltage(power);
    }

    public void setBottomMotor(double power) {
        bottomShooterMotor.setVoltage(power);
    }


    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.topShooterVelocityRotPerMin = topEncoder.getVelocity();
        inputs.bottomShooterVelocityRotPerMin = bottomEncoder.getVelocity();
        inputs.topShooterSupplyVoltage = topShooterMotor.getBusVoltage();
        inputs.bottomShooterSupplyVoltage = topShooterMotor.getBusVoltage();
        inputs.topShooterAmps = topShooterMotor.getOutputCurrent();
        inputs.bottomShooterAmps = topShooterMotor.getOutputCurrent();

    }
}
