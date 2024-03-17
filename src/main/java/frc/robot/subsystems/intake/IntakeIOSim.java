package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

    private final DigitalInput indexerBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INDEXER_BEAM_BRAKE_DIO_PORT);
    private final DigitalInput intakeBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INTAKE_BEAM_BRAKE_DIO_PORT);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.indexerBeamBrake = !indexerBeamBrake.get(); // true == game piece
        inputs.intakeBeamBrake = !intakeBeamBrake.get(); // true == game piece
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {}

    @Override
    public void setIndexerMotorPercentage(double percent) {}
}
