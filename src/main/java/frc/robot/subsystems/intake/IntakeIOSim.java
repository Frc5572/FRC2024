package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.sim.SimulatedPumbaa;

/**
 * Intake IO Sim
 */
public class IntakeIOSim implements IntakeIO {

    private FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 1, 0.025), DCMotor.getNEO(2), 0.0);
    private FlywheelSim indexerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 1, 0.025), DCMotor.getFalcon500(1), 0.0);

    private double intakeAppliedVolts = 0.0;
    private double indexerAppliedVolts = 0.0;

    private final SimulatedPumbaa pumbaa;

    /**
     * Intake IO Layer with simulated motors and sensors
     *
     * @param pumbaa Simulated Robot Viz
     */
    public IntakeIOSim(SimulatedPumbaa pumbaa) {
        this.pumbaa = pumbaa;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        intakeSim.update(LoggedRobot.defaultPeriodSecs);
        indexerSim.update(LoggedRobot.defaultPeriodSecs);

        inputs.intakeBeamBrake = pumbaa.lowerBeamBreak(); // true == game piece
        inputs.intakeRPM = intakeSim.getAngularVelocityRPM();
        inputs.indexerBeamBrake = pumbaa.upperBeamBreak(); // true == game piece
        inputs.indexerRPM = indexerSim.getAngularVelocityRPM();
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        intakeAppliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
        intakeSim.setInputVoltage(intakeAppliedVolts);
        pumbaa.setIntake(percent);
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        indexerAppliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
        indexerSim.setInputVoltage(indexerAppliedVolts);
        pumbaa.setIndexer(percent);
    }
}
