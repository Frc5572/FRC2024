package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants;

/**
 * Class for ShooterVortex
 */
public class ShooterVortex implements ShooterIO {
    public final SparkFlex topShooterMotor =
        new SparkFlex(Constants.Motors.Shooter.SHOOTER_TOP_ID, MotorType.kBrushless);
    public final SparkFlex bottomShooterMotor =
        new SparkFlex(Constants.Motors.Shooter.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
    private RelativeEncoder topEncoder = topShooterMotor.getEncoder();
    private RelativeEncoder bottomEncoder = bottomShooterMotor.getEncoder();

    /**
     * Constructor Shooter Subsystem - sets motor and encoder preferences
     */
    public ShooterVortex() {
        SparkBaseConfig TopConfig = new SparkFlexConfig();
        SparkBaseConfig BottomConfig = new SparkFlexConfig();
        BottomConfig.inverted(true).idleMode(IdleMode.kCoast).voltageCompensation(12);
        TopConfig.inverted(false).idleMode(IdleMode.kCoast).voltageCompensation(12);


        TopConfig.encoder.positionConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        TopConfig.encoder.velocityConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        BottomConfig.encoder.positionConversionFactor(Constants.ShooterConstants.GEAR_RATIO);
        BottomConfig.encoder.velocityConversionFactor(Constants.ShooterConstants.GEAR_RATIO);

        topShooterMotor.configure(TopConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        bottomShooterMotor.configure(BottomConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        // topShooterMotor.setSmartCurrentLimit(20);
        // bottomShooterMotor.setSmartCurrentLimit(20);
        // topShooterMotor.enableVoltageCompensation(12);
        // bottomShooterMotor.enableVoltageCompensation(12);
        // gear ratio 31:16
        // bottomShooterMotor.burnFlash();
        // topShooterMotor.burnFlash();

        // thread.start();
    }

    public void setTopMotor(double power) {
        // topShooterMotorVoltage = power;
        topShooterMotor.setVoltage(power);
    }

    public void setBottomMotor(double power) {
        // bottomShooterMotorVoltage = power;
        bottomShooterMotor.setVoltage(power);
    }


    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.topShooterVelocityRotPerMin = topEncoder.getVelocity();
        inputs.bottomShooterVelocityRotPerMin = bottomEncoder.getVelocity();
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
