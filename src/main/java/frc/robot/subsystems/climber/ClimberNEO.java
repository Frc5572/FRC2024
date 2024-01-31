package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

/**
 * Subsystem for physical climber.
 */
public class ClimberNEO implements ClimberIO {
    public final CANSparkMax leftMotor =
        new CANSparkMax(Constants.Motors.Climber.LEFT_MOTOR_ID, MotorType.kBrushless);
    public final CANSparkMax rightMotor =
        new CANSparkMax(Constants.Motors.Climber.RIGHT_MOTOR_ID, MotorType.kBrushless);

    public ClimberNEO() {
        rightMotor.follow(leftMotor);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.climberLeftMotorVoltage = leftMotor.get();
        inputs.climberLeftMotorAmp = leftMotor.getOutputCurrent();
        inputs.climberRightMotorVoltage = rightMotor.get();
        inputs.climberRightMotorAmp = rightMotor.getOutputCurrent();
    }

    @Override
    public void setClimberVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }
}
