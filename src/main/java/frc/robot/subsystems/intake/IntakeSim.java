package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeSim implements IntakeIO {

    private final DigitalInput indexerBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INDEXER_BEAM_BRAKE_DIO_PORT);
    private final DigitalInput intakeBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INTAKE_BEAM_BRAKE_DIO_PORT);

    public IntakeSim() {}

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
}
