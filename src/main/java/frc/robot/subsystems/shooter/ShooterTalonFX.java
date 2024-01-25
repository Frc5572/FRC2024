package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ShooterTalonFX implements ShooterIO {
    public final TalonFX shooterTopMotor = new TalonFX(Constants.Shooter.shooterTopId);
    public final TalonFX shooterBottomMotor = new TalonFX(Constants.Shooter.shooterBottomId);

    public void setTopMotor(double power) {
        shooterTopMotor.set(power);
    }

    public void setBottomMotor(double power) {
        shooterBottomMotor.set(power);
    }
}
