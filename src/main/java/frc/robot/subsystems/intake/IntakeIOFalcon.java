package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Intake IO Layer with real motors and sensors
 */
public class IntakeIOFalcon implements IntakeIO {

    private final SparkMax intakeMotorLeft =
        new SparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID_LEFT, MotorType.kBrushless);
    private final SparkMax intakeMotorRight =
        new SparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID_RIGHT, MotorType.kBrushless);
    public final RelativeEncoder intakeRelativeEnc =
        intakeMotorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    private final TalonFX indexerMotor = new TalonFX(Constants.Motors.Intake.INDEXER_MOTOR_ID);

    private final DutyCycleOut indexerDutyCycleOut = new DutyCycleOut(0);
    private final TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    private final DigitalInput indexerBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INDEXER_BEAM_BRAKE_DIO_PORT);
    private final DigitalInput intakeBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INTAKE_BEAM_BRAKE_DIO_PORT);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOFalcon() {
        intakeMotorLeft.restoreFactoryDefaults();
        intakeMotorRight.restoreFactoryDefaults();
        intakeMotorLeft.setInverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED);
        intakeMotorRight.setInverted(false);
        intakeMotorLeft.setIdleMode(IdleMode.kCoast);
        intakeMotorRight.setIdleMode(IdleMode.kCoast);
        intakeMotorLeft.setSmartCurrentLimit(40);
        intakeMotorRight.setSmartCurrentLimit(40);
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
        inputs.indexerBeamBrake = !indexerBeamBrake.get(); // true == game piece
        inputs.intakeBeamBrake = !intakeBeamBrake.get(); // true == game piece
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
