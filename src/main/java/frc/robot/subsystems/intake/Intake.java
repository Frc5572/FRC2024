package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Intake Subsystem
 */
public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeInputsAutoLogged intakeAutoLogged = new IntakeInputsAutoLogged();

    public Trigger noteInIntake = new Trigger(() -> getintakeBeamBrakeStatus()).debounce(0.25,
        Debouncer.DebounceType.kRising);
    public Trigger intakeActive = new Trigger(() -> getIntakeRPM() > 0);

    public Intake(IntakeIO io) {
        this.io = io;
        io.updateInputs(intakeAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(intakeAutoLogged);
        Logger.processInputs("Intake", intakeAutoLogged);
    }

    /**
     * Set the power of both intake motors
     *
     * @param percentage 0-1 power for the intake motors
     */
    public void setIntakeMotor(double percentage) {
        Logger.recordOutput("/Intake/Intake Percentage", percentage);
        io.setIntakeMotorPercentage(percentage);
    }

    /**
     * Get the status of the intake beam brake.
     *
     * @return True if beam brake is broken, False if open
     */
    public boolean getintakeBeamBrakeStatus() {
        return intakeAutoLogged.intakeBeamBrake;
    }

    public double getIntakeRPM() {
        return intakeAutoLogged.intakeRPM;
    }
}
