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

    private double topShooterMotorVoltage;
    private double bottomShooterMotorVoltage;
    private double topShooterVelocityRotPerMin = 0.0;
    private double bottomShooterVelocityRotPerMin = 0.0;

    private Thread thread = new Thread(() -> {
        while (true) {
            topShooterMotor.setVoltage(topShooterMotorVoltage);
            bottomShooterMotor.setVoltage(bottomShooterMotorVoltage);
            topShooterVelocityRotPerMin = topEncoder.getVelocity();
            bottomShooterVelocityRotPerMin = bottomEncoder.getVelocity();
        }
    });

    /**
     * Constructor Shooter Subsystem - sets motor and encoder preferences
     */
    public ShooterVortex() {
        topShooterMotor.setIdleMode(IdleMode.kCoast);
        bottomShooterMotor.setIdleMode(IdleMode.kCoast);
        topShooterMotor.setInverted(false);
        bottomShooterMotor.setInverted(true);
        topShooterMotor.enableVoltageCompensation(12);
        bottomShooterMotor.enableVoltageCompensation(12);
        // gear ratio 31:16
        topEncoder.setPositionConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        topEncoder.setVelocityConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        bottomEncoder.setPositionConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        bottomEncoder.setVelocityConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        bottomShooterMotor.burnFlash();
        topShooterMotor.burnFlash();

        thread.start();
    }

    public void setTopMotor(double power) {
        topShooterMotorVoltage = power;
    }

    public void setBottomMotor(double power) {
        bottomShooterMotorVoltage = power;
    }


    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.topShooterVelocityRotPerMin = this.topShooterVelocityRotPerMin;
        inputs.bottomShooterVelocityRotPerMin = this.bottomShooterVelocityRotPerMin;
        // inputs.topShooterPosition = topEncoder.getPosition();
        // inputs.bottomShooterPosition = bottomEncoder.getPosition();
        // inputs.topShooterSupplyVoltage = topShooterMotor.getBusVoltage();
        // inputs.bottomShooterSupplyVoltage = topShooterMotor.getBusVoltage();
        // inputs.topShooterAmps = topShooterMotor.getOutputCurrent();
        // inputs.bottomShooterAmps = topShooterMotor.getOutputCurrent();
        // inputs.topShooterPower = topShooterMotor.get();
        // inputs.bottomShooterPower = bottomShooterMotor.get();
        // inputs.topShooterTemp = topShooterMotor.getMotorTemperature();
        // inputs.bottomShooterTemp = bottomShooterMotor.getMotorTemperature();
    }
}
