package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * Intake Subsystem
 */
public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeInputsAutoLogged intakeAutoLogged = new IntakeInputsAutoLogged();

    // private GenericEntry beamBrake = RobotContainer.mainDriverTab.add("Have Note", false)
    // .withWidget(BuiltInWidgets.kBooleanBox).withPosition(9, 4).withSize(3, 2).getEntry();

    private GenericEntry haveNote =
        RobotContainer.mainDriverTab.add("Have Note", Color.kBlack.toHexString())
            .withWidget("Single Color View").withPosition(9, 4).withSize(3, 2).getEntry();

    public Intake(IntakeIO io) {
        this.io = io;
        io.updateInputs(intakeAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(intakeAutoLogged);
        Logger.processInputs("Intake", intakeAutoLogged);
        if (getIndexerBeamBrakeStatus() && getintakeBeamBrakeStatus()) {
            haveNote.setString(Color.kWhite.toHexString());
        } else if (getIndexerBeamBrakeStatus()) {
            haveNote.setString(Color.kPurple.toHexString());
        } else if (getintakeBeamBrakeStatus()) {
            haveNote.setString(Color.kGreen.toHexString());
        } else {
            haveNote.setString(Color.kBlack.toHexString());
        }
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
