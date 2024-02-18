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
        topShooterMotor.setInverted(false);
        bottomShooterMotor.setInverted(false);
        // gear ratio 31:16
        topEncoder.setPositionConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        topEncoder.setVelocityConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        bottomEncoder.setPositionConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        bottomEncoder.setVelocityConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
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
        inputs.topShooterPosition = topEncoder.getPosition();
        inputs.bottomShooterPosition = bottomEncoder.getPosition();
        inputs.topShooterSupplyVoltage = topShooterMotor.getBusVoltage();
        inputs.bottomShooterSupplyVoltage = topShooterMotor.getBusVoltage();
        inputs.topShooterAmps = topShooterMotor.getOutputCurrent();
        inputs.bottomShooterAmps = topShooterMotor.getOutputCurrent();
        inputs.topShooterPower = topShooterMotor.get();
        inputs.bottomShooterPower = bottomShooterMotor.get();
        inputs.topShooterTemp = topShooterMotor.getMotorTemperature();
        inputs.bottomShooterTemp = bottomShooterMotor.getMotorTemperature();
    }
}
