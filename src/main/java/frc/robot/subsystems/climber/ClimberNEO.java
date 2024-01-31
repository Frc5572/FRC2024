package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

/**
 * Subsystem for physical climber.
 */
public class ClimberNEO implements ClimberIO {
    public final CANSparkMax leftMotor =
        new CANSparkMax(Constants.Motors.Climber.LEFT_MOTOR_ID, MotorType.kBrushless);
    public final CANSparkMax rightMotor =
        new CANSparkMax(Constants.Motors.Climber.RIGHT_MOTOR_ID, MotorType.kBrushless);
    public final RelativeEncoder leftRelativeEncoder = leftMotor.getEncoder();
    public final RelativeEncoder RightRelativeEncoder = leftMotor.getEncoder();



    public ClimberNEO() {
        rightMotor.follow(leftMotor, true);
        leftRelativeEncoder.setPositionConversionFactor(25);
        RightRelativeEncoder.setPositionConversionFactor(25);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.climberLeftMotorVoltage = leftMotor.getBusVoltage();
        inputs.climberLeftMotorAmp = leftMotor.getOutputCurrent();
        inputs.climberRightMotorVoltage = rightMotor.getBusVoltage();
        inputs.climberRightMotorAmp = rightMotor.getOutputCurrent();
        inputs.leftMotorEncoderValue = leftRelativeEncoder.getPosition();
        inputs.rightMotorEncoderValue = RightRelativeEncoder.getPosition();

    }

    @Override
    public void setClimberVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }
}
