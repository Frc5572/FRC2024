package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class ClimberNEO implements ClimberIO {
    public final CANSparkMax climber1 =
        new CANSparkMax(Constants.Motors.Climber.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    public final CANSparkMax climber2 =
        new CANSparkMax(Constants.Motors.Climber.CLIMBER_MOTOR_ID2, MotorType.kBrushless);

    public ClimberNEO() {
        climber2.follow(climber1);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {

    }

    @Override
    public void setClimberVoltage(double voltage) {
        climber1.setVoltage(voltage);
    }
}
