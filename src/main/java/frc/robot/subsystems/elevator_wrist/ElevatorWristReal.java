package frc.robot.subsystems.elevator_wrist;


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

public class ElevatorWristReal implements ElevatorWristIO {
    public final TalonFX elevatorMotor = new TalonFX(Constants.Motors.ElevatorWrist.TALON_ID);

    public final Encoder elevatorRelativeEnc =
        new Encoder(Constants.ElevatorWristConstants.ELEVATOR_ENC_CHANNEL_A,
            Constants.ElevatorWristConstants.ELEVATOR_ENC_CHANNEL_B);

    public final CANSparkMax wristMotor =
        new CANSparkMax(Constants.Motors.ElevatorWrist.NEO_ID, MotorType.kBrushless);
    public final DigitalInput topLimitSwitch =
        new DigitalInput(Constants.ElevatorWristConstants.TOP_LIMIT_SWITCH_PORT);
    public final DigitalInput bottomLimitSwitch =
        new DigitalInput(Constants.ElevatorWristConstants.BOTTOM_LIMIT_SWITCH_PORT);

    public final AbsoluteEncoder wristAbsoluteEnc;

    private VoltageOut voltage = new VoltageOut(0);

    public ElevatorWristReal() {
        wristAbsoluteEnc = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

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
