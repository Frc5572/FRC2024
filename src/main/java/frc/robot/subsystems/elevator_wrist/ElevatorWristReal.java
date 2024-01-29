package frc.robot.subsystems.elevator_wrist;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

/**
 * Elevator and wrist real robot class
 */
public class ElevatorWristReal implements ElevatorWristIO {
    public final TalonFX elevatorMotor = new TalonFX(Constants.Motors.ElevatorWrist.TALON_ID);
    private final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
    public final Encoder elevatorRelativeEnc =
        new Encoder(Constants.ElevatorWristConstants.Sensors.ELEVATOR_ENC_CHANNEL_A,
            Constants.ElevatorWristConstants.Sensors.ELEVATOR_ENC_CHANNEL_B);

    public final CANSparkMax wristMotor =
        new CANSparkMax(Constants.Motors.ElevatorWrist.NEO_ID, MotorType.kBrushless);
    public final DigitalInput topLimitSwitch =
        new DigitalInput(Constants.ElevatorWristConstants.Sensors.TOP_LIMIT_SWITCH_PORT);
    public final DigitalInput bottomLimitSwitch =
        new DigitalInput(Constants.ElevatorWristConstants.Sensors.BOTTOM_LIMIT_SWITCH_PORT);

    public final AbsoluteEncoder wristAbsoluteEnc;

    private VoltageOut voltage = new VoltageOut(0);

    /**
     * Constructor for elevator wrist real class
     */
    public ElevatorWristReal() {
        wristAbsoluteEnc = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        wristAbsoluteEnc.setPositionConversionFactor(1); // Need to update with Gear ratio

        elevatorMotorConfig.Feedback.SensorToMechanismRatio = 25;
        elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setInverted(false);

        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setInverted(false);
    }


    @Override
    public void updateInputs(ElevatorWristInputs inputs) {
        inputs.topLimitSwitch = topLimitSwitch.get();
        inputs.bottomLimitSwitch = bottomLimitSwitch.get();
        inputs.elevatorRelativeEncRawValue = elevatorRelativeEnc.get();
        inputs.wristAbsoluteEncRawValue = wristAbsoluteEnc.getPosition();
        inputs.elevatorMotorSupplyVoltage = elevatorMotor.getSupplyVoltage().getValueAsDouble();
        inputs.elevatorMotorMotorVoltage = elevatorMotor.getMotorVoltage().getValueAsDouble();
        inputs.wristMotorVoltage = wristMotor.getBusVoltage();
        inputs.wristMotorAmp = wristMotor.getOutputCurrent();
    }

    @Override
    public void setElevatorVoltage(double v) {
        elevatorMotor.setControl(voltage.withOutput(v));
    }

    @Override
    public void setWristVoltage(double v) {
        wristMotor.setVoltage(v);
    }
}
