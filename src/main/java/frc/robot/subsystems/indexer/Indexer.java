package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Intake Subsystem
 */
public class Indexer extends SubsystemBase {
    private IndexerIO io;
    private IndexerInputsAutoLogged indexerAutoLogged = new IndexerInputsAutoLogged();

    // private GenericEntry beamBrake = RobotContainer.mainDriverTab.add("Have Note", false)
    // .withWidget(BuiltInWidgets.kBooleanBox).withPosition(9, 4).withSize(3, 2).getEntry();


    public Trigger noteInIndexer = new Trigger(() -> getIndexerBeamBrakeStatus()).debounce(0.25,
        Debouncer.DebounceType.kRising);
    public Trigger noteNotInIndexer = new Trigger(() -> !getIndexerBeamBrakeStatus());
    public Trigger indexerActive = new Trigger(() -> getIndexerRPM() > 0);

    public Indexer(IndexerIO io) {
        this.io = io;
        io.updateInputs(indexerAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(indexerAutoLogged);
        Logger.processInputs("Indexer", indexerAutoLogged);
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
        return indexerAutoLogged.indexerBeamBrake;
    }


    public double getIndexerRPM() {
        return indexerAutoLogged.indexerRPM;
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
