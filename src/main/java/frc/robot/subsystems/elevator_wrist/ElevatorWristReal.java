package frc.robot.subsystems.elevator_wrist;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Elevator and wrist real robot class
 */
public class ElevatorWristReal implements ElevatorWristIO {
    public final CANSparkMax leftElevatorMotor =
        new CANSparkMax(Constants.Motors.ElevatorWrist.ELEVATOR_LEFT_NEO_ID, MotorType.kBrushless);
    public final CANSparkMax rightElevatorMotor =
        new CANSparkMax(Constants.Motors.ElevatorWrist.ELEVATOR_RIGHT_NEO_ID, MotorType.kBrushless);
    public final CANSparkMax wristMotor =
        new CANSparkMax(Constants.Motors.ElevatorWrist.WRIST_NEO_ID, MotorType.kBrushless);
    public final DigitalInput topLimitSwitch =
        new DigitalInput(Constants.ElevatorWristConstants.Sensors.TOP_LIMIT_SWITCH_PORT);
    public final DigitalInput bottomLimitSwitch =
        new DigitalInput(Constants.ElevatorWristConstants.Sensors.BOTTOM_LIMIT_SWITCH_PORT);

    public final AbsoluteEncoder wristAbsoluteEnc = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    public final RelativeEncoder leftElevatorRelativeEnc =
        leftElevatorMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    public final RelativeEncoder rightElevatorRelativeEnc =
        rightElevatorMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);


    /**
     * Constructor for elevator wrist real class
     */
    public ElevatorWristReal() {
        rightElevatorMotor.restoreFactoryDefaults();
        rightElevatorRelativeEnc.setPositionConversionFactor(60);
        rightElevatorMotor.setIdleMode(IdleMode.kBrake);
        rightElevatorMotor.setInverted(true);
        leftElevatorMotor.restoreFactoryDefaults();
        leftElevatorRelativeEnc.setPositionConversionFactor(60);
        leftElevatorMotor.setIdleMode(IdleMode.kBrake);
        leftElevatorMotor.setInverted(false);
        wristAbsoluteEnc.setPositionConversionFactor(1);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setInverted(false);

        leftElevatorMotor.burnFlash();
        rightElevatorMotor.burnFlash();
    }


    @Override
    public void updateInputs(ElevatorWristInputs inputs) {
        inputs.topLimitSwitch = topLimitSwitch.get();
        inputs.bottomLimitSwitch = bottomLimitSwitch.get();
        inputs.rightElevatorRelativeEncRawValue = rightElevatorRelativeEnc.getPosition();
        inputs.leftElevatorRelativeEncRawValue = leftElevatorRelativeEnc.getPosition();
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
        rightElevatorMotor.setVoltage(v);
        leftElevatorMotor.setVoltage(v);
        // org.littletonrobotics.junction.Logger.recordOutput("elevator/voltage", v);
    }

    @Override
    public void setWristVoltage(double v) {
        wristMotor.setVoltage(v);
    }
}
