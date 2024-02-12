package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Intake IO Layer with real motors and sensors
 */
public class IntakeIOFalcon implements IntakeIO {

    private final TalonFX intakeMotor =
        new TalonFX(Constants.Motors.Intake.INTAKE_MOTOR_ID, "canivore");
    private final TalonFX indexerMotor = new TalonFX(Constants.Motors.Intake.INDEXER_MOTOR_ID);

    private final DutyCycleOut intakeDutyCycleOut = new DutyCycleOut(0);
    private final DutyCycleOut indexerDutyCycleOut = new DutyCycleOut(0);

    private final DigitalInput beamBrake = new DigitalInput(8);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOFalcon() {
        intakeMotor.setInverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        indexerMotor.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeSupplyVoltage = intakeMotor.getSupplyVoltage().getValueAsDouble();
        inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakeAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
        inputs.intakeRPM = intakeMotor.getVelocity().getValueAsDouble();
        inputs.indexerSupplyVoltage = indexerMotor.getSupplyVoltage().getValueAsDouble();
        inputs.indexerMotorVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.indexerAmps = indexerMotor.getStatorCurrent().getValueAsDouble();
        inputs.indexerRPM = indexerMotor.getVelocity().getValueAsDouble();
        inputs.sensorStatus = beamBrake.get(); // true == no game piece
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        intakeMotor.setControl(intakeDutyCycleOut.withOutput(percent));
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        indexerMotor.setControl(indexerDutyCycleOut.withOutput(percent));
    }
}
