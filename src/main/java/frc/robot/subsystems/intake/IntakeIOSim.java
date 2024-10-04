package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Intake IO Layer with real motors and sensors
 */
public class IntakeIOSim implements IntakeIO {

    private FlywheelSim inputSim = new FlywheelSim(DCMotor.getNEO(2), 1, 0.025);
    private double turnAppliedVolts = 0.0;

    private final DigitalInput intakeBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INTAKE_BEAM_BRAKE_DIO_PORT);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOSim() {

    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputSim.update(Constants.loopPeriodSecs);
        inputs.intakeBeamBrake = !intakeBeamBrake.get(); // true == game piece
        inputs.intakeRPM = inputSim.getAngularVelocityRPM();
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        turnAppliedVolts = MathUtil.clamp(percent * 12.0, -12.0, 12.0);
        inputSim.setInputVoltage(turnAppliedVolts);
    }
}
