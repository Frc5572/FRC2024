package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Intake Subsystem
 */
public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeInputsAutoLogged intakeAutoLogged = new IntakeInputsAutoLogged();

    public Trigger noteInIntake = new Trigger(() -> getintakeBeamBrakeStatus()).debounce(0.25,
        Debouncer.DebounceType.kRising);
    public Trigger noteInIndexer = new Trigger(() -> getIndexerBeamBrakeStatus()).debounce(0.25,
        Debouncer.DebounceType.kRising);
    public Trigger noteNotInIndexer = new Trigger(() -> !getIndexerBeamBrakeStatus());
    public Trigger intakeActive = new Trigger(() -> getIntakeRPM() > 0);

    public GenericEntry haveNote = RobotContainer.mainDriverTab
        .add("Have Note", Constants.LEDConstants.NO_NOTE_COLOR.toHexString())
        .withWidget("Single Color View").withPosition(9, 4).withSize(3, 2).getEntry();

    public Intake(IntakeIO io) {
        this.io = io;
        io.updateInputs(intakeAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(intakeAutoLogged);
        Logger.processInputs("Intake", intakeAutoLogged);
        // if (getIndexerBeamBrakeStatus() && getintakeBeamBrakeStatus()) {
        // haveNote.setString(Constants.LEDConstants.ALERT_COLOR.toHexString());
        // } else if (getIndexerBeamBrakeStatus()) {
        // haveNote.setString(Constants.LEDConstants.INDEXER_COLOR.toHexString());
        // } else if (getintakeBeamBrakeStatus()) {
        // haveNote.setString(Constants.LEDConstants.INTAKE_COLOR.toHexString());
        // } else {
        // haveNote.setString(noNote);
        // }
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
     * Set the power for the indexer motor
     *
     * @param percentage 0-1 power for the indexer motor
     */
    public void setIndexerMotor(double percentage) {
        Logger.recordOutput("/Intake/Indexer Percentage", percentage);
        io.setIndexerMotorPercentage(percentage);
    }

    public double getIntakeRPM() {
        return intakeAutoLogged.intakeRPM;
    }

    /**
     * Get the status of the indexer beam brake.
     *
     * @return True if beam brake is broken, False if open
     */
    public boolean getIndexerBeamBrakeStatus() {
        return intakeAutoLogged.indexerBeamBrake;
    }

    /**
     * Get the status of the intake beam brake.
     *
     * @return True if beam brake is broken, False if open
     */
    public boolean getintakeBeamBrakeStatus() {
        return intakeAutoLogged.intakeBeamBrake;
    }

    /**
     * Command to run the intake motor and indexer until the sensor trips
     *
     * @return {@link Command} to run the intake and indexer motors
     */
    public Command runIntakeMotor(double intakeSpeed, double indexerSpeed) {
        return Commands.startEnd(() -> {
            setIntakeMotor(intakeSpeed);
            setIndexerMotor(indexerSpeed);
        }, () -> {
            setIntakeMotor(0);
            setIndexerMotor(0);
        }, this).until(() -> getIndexerBeamBrakeStatus()).unless(() -> getIndexerBeamBrakeStatus());
    }

    /**
     * Command to run the intake motor and indexer until the sensor trips
     *
     * @return {@link Command} to run the intake and indexer motors
     */
    public Command runIntakeMotorNonStop(double intakeSpeed, double indexerSpeed) {
        return Commands.startEnd(() -> {
            setIntakeMotor(intakeSpeed);
            setIndexerMotor(indexerSpeed);
        }, () -> {
            setIntakeMotor(0);
            setIndexerMotor(0);
        }, this);
    }

    /**
     * Command to run the indexer
     *
     * @return {@link Command} to run the indexer motors
     */
    public Command runIndexerMotor(double speed) {
        return Commands.startEnd(() -> {
            setIndexerMotor(speed);
        }, () -> {
            setIndexerMotor(0);
        }, this);
    }
}
