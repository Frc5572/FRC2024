package frc.robot.subsystems.elevator_wrist;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

    public final RelativeEncoder wristAbsoluteEnc;
    public final RelativeEncoder elevatorAbsoluteEnc;


    /**
     * Constructor for elevator wrist real class
     */
    public ElevatorWristReal() {
        wristAbsoluteEnc = wristMotor.getEncoder();
        wristAbsoluteEnc.setPositionConversionFactor(1);

        elevatorAbsoluteEnc = elevatorMotor.getEncoder();
        elevatorAbsoluteEnc.setPositionConversionFactor(25);

        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setInverted(false);

        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setInverted(false);
    }


    @Override
    public void updateInputs(ElevatorWristInputs inputs) {
        inputs.topLimitSwitch = topLimitSwitch.get();
        inputs.bottomLimitSwitch = bottomLimitSwitch.get();
        inputs.elevatorAbsoluteEncRawValue = elevatorAbsoluteEnc.getPosition();
        inputs.wristAbsoluteEncRawValue = wristAbsoluteEnc.getPosition();
        inputs.elevatorMotorSupplyVoltage = elevatorMotor.getBusVoltage();
        inputs.elevatorMotorMotorVoltage = elevatorMotor.getOutputCurrent();
        inputs.wristMotorVoltage = wristMotor.getBusVoltage();
        inputs.wristMotorAmp = wristMotor.getOutputCurrent();
    }

    @Override
    public void setElevatorVoltage(double v) {
        elevatorMotor.setVoltage(v);
    }

    @Override
    public void setWristVoltage(double v) {
        wristMotor.setVoltage(v);
    }
}
