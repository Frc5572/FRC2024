package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

    private FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNEO(2), 1, 0.025);
    private FlywheelSim indexerSim = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.025);

    private double intakeAppliedVolts = 0.0;
    private double indexerAppliedVolts = 0.0;


    private final DigitalInput indexerBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INDEXER_BEAM_BRAKE_DIO_PORT);
    private final DigitalInput intakeBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INTAKE_BEAM_BRAKE_DIO_PORT);


    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOSim() {

    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        intakeSim.update(LoggedRobot.defaultPeriodSecs);
        indexerSim.update(LoggedRobot.defaultPeriodSecs);

        inputs.intakeBeamBrake = !intakeBeamBrake.get(); // true == game piece
        inputs.intakeRPM = intakeSim.getAngularVelocityRPM();
        inputs.indexerBeamBrake = !indexerBeamBrake.get(); // true == game piece
        inputs.indexerRPM = indexerSim.getAngularVelocityRPM();
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        intakeAppliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        indexerAppliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
        indexerSim.setInputVoltage(indexerAppliedVolts);
    }
}
