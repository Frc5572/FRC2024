package frc.robot.subsystems.elevator_wrist;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

/**
 * Elevator and wrist real robot class
 */
public class ElevatorWristReal implements ElevatorWristIO {
    public final SparkMax leftElevatorMotor =
        new SparkMax(Constants.Motors.ElevatorWrist.ELEVATOR_LEFT_NEO_ID, MotorType.kBrushless);
    public final SparkMax rightElevatorMotor =
        new SparkMax(Constants.Motors.ElevatorWrist.ELEVATOR_RIGHT_NEO_ID, MotorType.kBrushless);
    public final SparkMax wristMotor =
        new SparkMax(Constants.Motors.ElevatorWrist.WRIST_NEO_ID, MotorType.kBrushless);

    public final AbsoluteEncoder wristAbsoluteEnc = wristMotor.getAbsoluteEncoder();
    public final RelativeEncoder leftElevatorRelativeEnc = leftElevatorMotor.getEncoder();
    public final RelativeEncoder rightElevatorRelativeEnc = rightElevatorMotor.getEncoder();
    public final SparkMaxConfig leftELIConfig = new SparkMaxConfig();
    public final SparkMaxConfig rightELIConfig = new SparkMaxConfig();
    public final SparkMaxConfig wristConfig = new SparkMaxConfig();



    /**
     * Constructor for elevator wrist real class
     */
    public ElevatorWristReal() {
        // left elevator motor config
        leftELIConfig.inverted(true).idleMode(IdleMode.kBrake);
        leftELIConfig.encoder.positionConversionFactor(60);
        leftElevatorMotor.configure(leftELIConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        // right elevator motor config
        rightELIConfig.inverted(false).idleMode(IdleMode.kBrake);
        rightELIConfig.encoder.positionConversionFactor(60);
        rightElevatorMotor.configure(rightELIConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        // wrist motor config
        wristConfig.inverted(false).idleMode(IdleMode.kBrake);
        wristConfig.encoder.positionConversionFactor(1);
        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }


    @Override
    public void updateInputs(ElevatorWristInputs inputs) {
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
