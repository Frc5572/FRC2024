package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Intake IO Layer with real motors and sensors
 */
public class IndexerIOFalcon implements IndexerIO {

    private final TalonFX indexerMotor = new TalonFX(Constants.Motors.Intake.INDEXER_MOTOR_ID);

    private final DutyCycleOut indexerDutyCycleOut = new DutyCycleOut(0);
    private final TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    private final DigitalInput indexerBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INDEXER_BEAM_BRAKE_DIO_PORT);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IndexerIOFalcon() {
        indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        indexerMotor.getConfigurator().apply(indexerConfig);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        // inputs.intakeSupplyVoltage = intakeMotorLeft.getBusVoltage();
        // inputs.intakeAmps = intakeMotorLeft.getOutputCurrent();
        // inputs.intakeRPM = intakeRelativeEnc.getVelocity();
        // inputs.indexerSupplyVoltage = indexerMotor.getSupplyVoltage().getValueAsDouble();
        // inputs.indexerMotorVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        // inputs.indexerAmps = indexerMotor.getSupplyCurrent().getValueAsDouble();
        // inputs.indexerRPM = indexerMotor.getVelocity().getValueAsDouble();
        inputs.indexerBeamBrake = !indexerBeamBrake.get(); // true == game piece
        inputs.indexerRPM = 0;
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        indexerMotor.setControl(indexerDutyCycleOut.withOutput(percent));
    }
}
