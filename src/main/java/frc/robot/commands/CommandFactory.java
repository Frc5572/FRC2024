package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * File to create commands using factories
 */
public class CommandFactory {

    /**
     * Checks to make sure all preconditions are met before running the intake
     *
     * @param intake Intake subsystem
     * @param elevatorWrist Elevator and Wrist subsystem
     * @return Returns a command
     */
    public static Command runIntake(Intake intake, Indexer indexer, ElevatorWrist elevatorWrist) {
        BooleanSupplier sensor = () -> indexer.getIndexerBeamBrakeStatus();
        Command moveElevatorWrist =
            elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE);
        Command runIntakeIndexer =
            runIntakeMotor(intake, indexer, 0, Constants.IntakeConstants.INDEX_MOTOR_FORWARD);
        return moveElevatorWrist.andThen(runIntakeIndexer).unless(sensor);
    }

    /**
     * Wrist follows the speaker until it is met, then it shoots
     *
     * @param shooter Shooter subsystem
     * @param indexer Indexer subsystem
     * @return Returns a command
     */
    public static Command shootSpeaker(Shooter shooter, Indexer indexer) {
        Command runIndexer = indexer.runIndexerMotor(1);
        Command runshooter = shooter.shootSpeaker();
        Command readytoShoot = Commands.waitUntil(() -> shooter.readyToShoot());
        return runshooter.alongWith(readytoShoot.withTimeout(1).andThen(runIndexer));
    }

    /**
     * Runs intake, indexer and shooter all at once.
     *
     * @param shooter Shooter subsystem
     * @param intake Intake subsystem
     * @param indexer Indexer Subsystem
     * @return Returns a command
     */
    public static Command passThroughShoot(Shooter shooter, Intake intake, Indexer indexer) {
        Command runshooter = shooter.shootSpeaker();
        Command readytoShoot = Commands.waitUntil(() -> shooter.readyToShoot());
        return runshooter.alongWith(
            readytoShoot.withTimeout(1).andThen(runIntakeMotorNonStop(intake, indexer, 0, 1)
                .withTimeout(1).andThen(runIntakeMotorNonStop(intake, indexer, 1, 1))));
    }

    /**
     * Command to spit out the notes
     *
     * @param shooter Shooter Subsystem
     * @param indexer Indexer Subsystem
     * @return Command
     */
    public static Command spit(Shooter shooter, Indexer indexer) {
        return shooter.spit().alongWith(indexer.runIndexerMotor(1.0));
    }

    /**
     * Command to run the intake and indexer at the proper speed to intake a note
     *
     * @param intake Intake Subsystem
     * @param indexer Indexer Subsystem
     * @return Command
     */
    public static Command intakeNote(Intake intake, Indexer indexer) {
        return runIntakeMotor(intake, indexer, 1, .2);
    }

    /**
     * Command to auto angle the wirst for the speaker opening based on distance from the speaker
     *
     * @param elevatorWrist Elevator Wrist Subsystem
     * @param swerveDrive Swerve Drive Subsystem
     * @return Command
     */
    public static Command autoAngleWristSpeaker(ElevatorWrist elevatorWrist, Swerve swerveDrive) {
        return elevatorWrist.followPosition(
            () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            () -> elevatorWrist.getAngleFromDistance(swerveDrive.getPose()));
    }

    /**
     * New intake command to prevent intaking past the intake when the elevator isn't home
     *
     * @param intake Intake Subsystem
     * @param indexer Indexer Subsystem
     * @param elevatorWrist Elevator Wrist Subsystem
     * @return Command
     */
    public static Command newIntakeCommand(Intake intake, Indexer indexer,
        ElevatorWrist elevatorWrist) {
        Command regularIntake = intakeNote(intake, indexer);
        Command altIntake = Commands
            .startEnd(() -> intake.setIntakeMotor(1), () -> intake.setIntakeMotor(0), intake)
            .until(intake.noteInIntake)
            .andThen(Commands.waitUntil(elevatorWrist.elevatorAtHome), intakeNote(intake, indexer));

        return Commands.either(regularIntake, altIntake, elevatorWrist.elevatorAtHome)
            .unless(indexer.noteInIndexer);
    }

    /**
     * Command Factory for Auto Specific Commands
     */
    public class Auto {

        /**
         * Command to run the indexer to shoot a note until .25 seconds after the beam brake is no
         * longer broken
         *
         * @param indexer Indexer Subsystem
         * @return Command
         */
        public static Command runIndexer(Indexer indexer) {
            return Commands.waitUntil(indexer.noteInIndexer.negate())
                .andThen(Commands.waitSeconds(.25)).deadlineWith(indexer.runIndexerMotor(1))
                .withTimeout(5);
        }

        /**
         * Command to run the indexer to shoot a note until .25 seconds after the beam brake is no
         * longer broken
         *
         * @param indexer Indexer Subsystem
         * @return Command
         */
        public static Command runIndexer(Indexer indexer, Shooter shooter) {
            return Commands.waitUntil(() -> shooter.readyToShoot()).withTimeout(2)
                .andThen(CommandFactory.Auto.runIndexer(indexer));
        }

        /**
         * Command to wait for Intake beam brake is tripped
         *
         * @param indexer Indexer Subsystem
         * @return Command
         */
        public static Command waitForIntake(Indexer indexer) {
            return Commands.waitUntil(() -> indexer.getIndexerBeamBrakeStatus());
        }

    }

    /**
     * Command to run the intake motor and indexer until the sensor trips
     *
     * @return {@link Command} to run the intake and indexer motors
     */
    public static Command runIntakeMotor(Intake intake, Indexer indexer, double intakeSpeed,
        double indexerSpeed) {
        return runIntakeMotorNonStop(intake, indexer, intakeSpeed, indexerSpeed)
            .until(indexer.noteInIndexer).unless(indexer.noteInIndexer);
    }

    /**
     * Command to run the intake motor and indexer until the sensor trips
     *
     * @return {@link Command} to run the intake and indexer motors
     */
    public static Command runIntakeMotorNonStop(Intake intake, Indexer indexer, double intakeSpeed,
        double indexerSpeed) {
        return Commands.startEnd(() -> {
            intake.setIntakeMotor(intakeSpeed);
            indexer.setIndexerMotor(indexerSpeed);
        }, () -> {
            intake.setIntakeMotor(0);
            indexer.setIndexerMotor(0);
        }, intake, indexer);
    }
}
