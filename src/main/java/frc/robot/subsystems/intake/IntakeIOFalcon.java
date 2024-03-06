package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Intake IO Layer with real motors and sensors
 */
public class IntakeIOFalcon implements IntakeIO {

    private final CANSparkMax intakeMotorLeft =
        new CANSparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID_LEFT, MotorType.kBrushless);
    private final CANSparkMax intakeMotorRight =
        new CANSparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID_RIGHT, MotorType.kBrushless);
    public final RelativeEncoder intakeRelativeEnc =
        intakeMotorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    private final TalonFX indexerMotor = new TalonFX(Constants.Motors.Intake.INDEXER_MOTOR_ID);

    private final DutyCycleOut indexerDutyCycleOut = new DutyCycleOut(0);
    private final DigitalInput beamBrake = new DigitalInput(8);
    private TalonFXConfiguration indexerConfig = new TalonFXConfiguration();

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOFalcon() {
        intakeMotorLeft.restoreFactoryDefaults();
        intakeMotorRight.restoreFactoryDefaults();
        intakeMotorLeft.setInverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED);
        intakeMotorLeft.setIdleMode(IdleMode.kCoast);
        intakeMotorRight.setInverted(false);
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerMotor.getConfigurator().apply(indexerConfig);

    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // inputs.intakeSupplyVoltage = intakeMotorLeft.getBusVoltage();
        // inputs.intakeAmps = intakeMotorLeft.getOutputCurrent();
        // inputs.intakeRPM = intakeRelativeEnc.getVelocity();
        // inputs.indexerSupplyVoltage = indexerMotor.getSupplyVoltage().getValueAsDouble();
        // inputs.indexerMotorVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        // inputs.indexerAmps = indexerMotor.getSupplyCurrent().getValueAsDouble();
        // inputs.indexerRPM = indexerMotor.getVelocity().getValueAsDouble();
        inputs.sensorStatus = beamBrake.get(); // true == no game piece
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        // Left ratio is 60:30
        // Right ratio is 32:30
        intakeMotorLeft.set(percent);
        intakeMotorRight.set(percent);
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        indexerMotor.setControl(indexerDutyCycleOut.withOutput(percent));
    }
}
