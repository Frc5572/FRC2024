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
    public final RelativeEncoder rightRelativeEncoder = rightMotor.getEncoder();


    /**
     * Constructor for climberNEO subsystem.
     */
    public ClimberNEO() {
        rightMotor.follow(leftMotor, true);
        leftRelativeEncoder.setPositionConversionFactor(25);
        rightRelativeEncoder.setPositionConversionFactor(25);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.climberLeftMotorVoltage = leftMotor.getBusVoltage();
        inputs.climberLeftMotorAmp = leftMotor.getOutputCurrent();
        inputs.climberLeftMotorTemp = leftMotor.getMotorTemperature();
        inputs.climberRightMotorVoltage = rightMotor.getBusVoltage();
        inputs.climberRightMotorAmp = rightMotor.getOutputCurrent();
        inputs.climberRightMotorTemp = rightMotor.getMotorTemperature();
        inputs.leftMotorEncoderValue = leftRelativeEncoder.getPosition();
        inputs.rightMotorEncoderValue = rightRelativeEncoder.getPosition();

    }

    @Override
    public void setLeftClimberVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    public void setRightClimberVoltage(double voltage) {
        rightMotor.setVoltage(voltage);
    }

    public void setLeftPower(double power) {
        leftMotor.set(power);
    }

    public void setRightPower(double power) {
        rightMotor.set(power);
    }
}
