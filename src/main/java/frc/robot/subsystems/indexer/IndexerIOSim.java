package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Intake IO Layer with real motors and sensors
 */
public class IndexerIOSim implements IndexerIO {

    private FlywheelSim indexerSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.025);
    private final DigitalInput indexerBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INDEXER_BEAM_BRAKE_DIO_PORT);
    private double turnAppliedVolts = 0.0;

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IndexerIOSim() {

    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        indexerSim.update(Constants.loopPeriodSecs);
        inputs.indexerBeamBrake = !indexerBeamBrake.get(); // true == game piece
        inputs.indexerRPM = indexerSim.getAngularVelocityRPM();
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        turnAppliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
        indexerSim.setInputVoltage(turnAppliedVolts);
    }
}
