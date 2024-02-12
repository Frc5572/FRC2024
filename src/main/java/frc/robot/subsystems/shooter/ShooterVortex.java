package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
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

    /**
     * Constructor Shooter Subsystem - sets motor and encoder preferences
     */
    public ShooterVortex() {
        topShooterMotor.setIdleMode(IdleMode.kCoast);
        bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        topEncoder.setPositionConversionFactor(31.0 / 16.0);
        topEncoder.setVelocityConversionFactor(31.0 / 16.0);
        bottomEncoder.setPositionConversionFactor(31.0 / 16.0);
        bottomEncoder.setVelocityConversionFactor(31.0 / 16.0);
        bottomShooterMotor.burnFlash();
        topShooterMotor.burnFlash();
    }

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
        inputs.topShooterPosition = topEncoder.getPosition();
        inputs.bottomShooterPosition = bottomEncoder.getPosition();
        inputs.topShooterPower = topShooterMotor.get();
        inputs.bottomShooterPower = bottomShooterMotor.get();
    }
}
