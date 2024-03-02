package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import frc.robot.Constants;

/**
 * Subsystem for physical climber.
 */
public class ClimberNEO implements ClimberIO {
    public final CANSparkMax leftMotor =
        new CANSparkMax(Constants.Motors.Climber.LEFT_MOTOR_ID, MotorType.kBrushless);
    public final CANSparkMax rightMotor =
        new CANSparkMax(Constants.Motors.Climber.RIGHT_MOTOR_ID, MotorType.kBrushless);
    public final RelativeEncoder leftRelativeEncoder =
        leftMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    public final RelativeEncoder rightRelativeEncoder =
        rightMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);


    /**
     * Constructor for climberNEO subsystem.
     */
    public ClimberNEO() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftRelativeEncoder.setPositionConversionFactor(25);
        rightRelativeEncoder.setPositionConversionFactor(25);
        rightMotor.setInverted(true);
        rightMotor.burnFlash();
        leftMotor.burnFlash();
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        // inputs.climberLeftMotorVoltage = leftMotor.getBusVoltage();
        // inputs.climberLeftMotorAmp = leftMotor.getOutputCurrent();
        // inputs.climberLeftMotorTemp = leftMotor.getMotorTemperature();
        // inputs.climberRightMotorVoltage = rightMotor.getBusVoltage();
        // inputs.climberRightMotorAmp = rightMotor.getOutputCurrent();
        // inputs.climberRightMotorTemp = rightMotor.getMotorTemperature();
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
