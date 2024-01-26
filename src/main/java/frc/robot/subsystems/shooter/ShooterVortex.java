package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class ShooterVortex implements ShooterIO {
    public final CANSparkFlex shooterTopMotor =
        new CANSparkFlex(Constants.Shooter.shooterTopId, MotorType.kBrushless);
    public final CANSparkFlex shooterBottomMotor =
        new CANSparkFlex(Constants.Shooter.shooterBottomId, MotorType.kBrushless);

    public void setTopMotor(double power) {
        shooterTopMotor.setVoltage(power);
    }

    public void setBottomMotor(double power) {
        shooterBottomMotor.setVoltage(power);
    }
}
