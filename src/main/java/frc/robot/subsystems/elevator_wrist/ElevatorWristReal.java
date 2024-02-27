package frc.robot.subsystems.elevator_wrist;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Elevator and wrist real robot class
 */
public class ElevatorWristReal implements ElevatorWristIO {
    public final CANSparkMax elevatorMotor =
        new CANSparkMax(Constants.Motors.ElevatorWrist.ELEVATOR_NEO_ID, MotorType.kBrushless);
    public final CANSparkMax wristMotor =
        new CANSparkMax(Constants.Motors.ElevatorWrist.WRIST_NEO_ID, MotorType.kBrushless);
    public final DigitalInput topLimitSwitch =
        new DigitalInput(Constants.ElevatorWristConstants.Sensors.TOP_LIMIT_SWITCH_PORT);
    public final DigitalInput bottomLimitSwitch =
        new DigitalInput(Constants.ElevatorWristConstants.Sensors.BOTTOM_LIMIT_SWITCH_PORT);

    public final AbsoluteEncoder wristAbsoluteEnc;
    public final RelativeEncoder elevatorRelativeEnc;


    /**
     * Constructor for elevator wrist real class
     */
    public ElevatorWristReal() {
        wristAbsoluteEnc = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        wristAbsoluteEnc.setPositionConversionFactor(1);

        elevatorRelativeEnc = elevatorMotor.getEncoder();
        elevatorRelativeEnc.setPositionConversionFactor(25);

        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(false);

        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setInverted(false);
    }


    @Override
    public void updateInputs(ElevatorWristInputs inputs) {
        inputs.topLimitSwitch = topLimitSwitch.get();
        inputs.bottomLimitSwitch = bottomLimitSwitch.get();
        inputs.elevatorRelativeEncRawValue = elevatorRelativeEnc.getPosition();
        inputs.wristAbsoluteEncRawValue = wristAbsoluteEnc.getPosition();
        // inputs.elevatorMotorSupplyVoltage = elevatorMotor.getBusVoltage();
        // inputs.elevatorMotorVoltage = elevatorMotor.getOutputCurrent();
        // inputs.elevatorMotorTemp = elevatorMotor.getMotorTemperature();
        // inputs.wristMotorVoltage = wristMotor.getBusVoltage();
        // inputs.wristMotorAmp = wristMotor.getOutputCurrent();
        // inputs.wristMotorTemp = wristMotor.getMotorTemperature();
    }

    @Override
    public void setElevatorVoltage(double v) {
        elevatorMotor.setVoltage(v);
    }

    @Override
    public void setWristVoltage(double v) {
        wristMotor.setVoltage(v);
    }

    @Override
    public void setWristPower(double power) {
        wristMotor.set(power);
    }

    @Override
    public void setElevatorPower(double power) {
        elevatorMotor.set(power);
    }
}
