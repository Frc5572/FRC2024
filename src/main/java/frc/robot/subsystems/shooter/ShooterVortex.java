package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

/**
 * Class for ShooterVortex
 */
public class ShooterVortex implements ShooterIO {
    public final CANSparkFlex shooterTopMotor =
        new CANSparkFlex(Constants.Motors.Shooter.shooterTopId, MotorType.kBrushless);
    public final CANSparkFlex shooterBottomMotor =
        new CANSparkFlex(Constants.Motors.Shooter.shooterBottomId, MotorType.kBrushless);
    private RelativeEncoder topEncoder;
    private RelativeEncoder bottomEncoder;

    public ShooterVortex() {
        topEncoder = shooterTopMotor.getEncoder();
        bottomEncoder = shooterBottomMotor.getEncoder();
    }

    public void setTopMotor(double power) {
        shooterTopMotor.setVoltage(power);
    }

    public void setBottomMotor(double power) {
        shooterBottomMotor.setVoltage(power);
    }

    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.topshooterVelocityRotPerSecond = topEncoder.getVelocity();
        inputs.bottomshooterVelocityRotPerSecond = bottomEncoder.getVelocity();
        inputs.topshooterSupplyVoltage = shooterTopMotor.getBusVoltage();
        inputs.bottomshooterSupplyVoltage = shooterBottomMotor.getBusVoltage();
        inputs.topshooterAmps = shooterTopMotor.getOutputCurrent();
        inputs.bottomshooterAmps = shooterBottomMotor.getOutputCurrent();
    }
}
