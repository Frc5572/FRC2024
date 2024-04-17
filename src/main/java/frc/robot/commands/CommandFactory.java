package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
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
    public static Command runIntake(Intake intake, ElevatorWrist elevatorWrist) {
        BooleanSupplier sensor = () -> intake.getIndexerBeamBrakeStatus();
        Command moveElevatorWrist =
            elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE);
        Command runIntakeIndexer =
            intake.runIntakeMotor(0, Constants.IntakeConstants.INDEX_MOTOR_FORWARD);
        return moveElevatorWrist.andThen(runIntakeIndexer).unless(sensor);
    }

    /**
     * Wrist follows the speaker until it is met, then it shoots
     *
     * @param shooter Shooter subsystem
     * @param intake Intake subsystem
     * @return Returns a command
     */
    public static Command shootSpeaker(Shooter shooter, Intake intake) {
        Command runIndexer = intake.runIndexerMotor(1);
        Command runshooter = shooter.shootSpeaker();
        Command readytoShoot = Commands.waitUntil(() -> shooter.readyToShoot());
        return runshooter.alongWith(readytoShoot.withTimeout(1).andThen(runIndexer));
    }

    /**
     * Runs intake, indexer and shooter all at once.
     *
     * @param shooter Shooter subsystem
     * @param intake Intake subsystem
     * @return Returns a command
     */
    public static Command passThroughShoot(Shooter shooter, Intake intake) {
        Command runshooter = shooter.shootSpeaker();
        Command readytoShoot = Commands.waitUntil(() -> shooter.readyToShoot());
        return runshooter
            .alongWith(readytoShoot.withTimeout(1).andThen(intake.runIntakeMotorNonStop(0, 1)
                .withTimeout(1).andThen(intake.runIntakeMotorNonStop(1, 1))));
    }

    /**
     * Command to spit out the notes
     *
     * @param shooter Shooter Subsystem
     * @param intake Intake Subsystem
     * @return Command
     */
    public static Command spit(Shooter shooter, Intake intake) {
        return shooter.spit().alongWith(intake.runIndexerMotor(1.0));
    }

    /**
     * Command to run the intake and indexer at the proper speed to intake a note
     *
     * @param intake Intake Subsystem
     * @return Command
     */
    public static Command intakeNote(Intake intake) {
        return intake.runIntakeMotor(1, .2);
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
     * @param elevatorWrist Elevator Wrist Subsystem
     * @return Command
     */
    public static Command newIntakeCommand(Intake intake, ElevatorWrist elevatorWrist) {
        Trigger noteInIntake = new Trigger(() -> intake.getintakeBeamBrakeStatus()).debounce(0.25,
            Debouncer.DebounceType.kRising);
        Command regularIntake = intakeNote(intake);
        Command altIntake = Commands
            .startEnd(() -> intake.setIntakeMotor(1), () -> intake.setIntakeMotor(0), intake)
            .until(noteInIntake)
            .andThen(Commands.waitUntil(() -> elevatorWrist.elevatorAtHome()), intakeNote(intake));

        return Commands.either(regularIntake, altIntake, () -> elevatorWrist.elevatorAtHome())
            .unless(() -> intake.getIndexerBeamBrakeStatus());
    }

    /**
     * Command Factory for Auto Specific Commands
     */
    public class Auto {

        /**
         * Command to run the indexer to shoot a note until .25 seconds after the beam brake is no
         * longer broken
         *
         * @param intake Intake Subsystem
         * @return Command
         */
        public static Command runIndexer(Intake intake) {
            return Commands.waitUntil(() -> !intake.getIndexerBeamBrakeStatus())
                .andThen(Commands.waitSeconds(.25)).deadlineWith(intake.runIndexerMotor(1))
                .withTimeout(5);
        }

        /**
         * Command to run the indexer to shoot a note until .25 seconds after the beam brake is no
         * longer broken
         *
         * @param intake Intake Subsystem
         * @return Command
         */
        public static Command runIndexer(Intake intake, Shooter shooter) {
            return Commands.waitUntil(() -> shooter.readyToShoot()).withTimeout(2)
                .andThen(CommandFactory.Auto.runIndexer(intake));
        }

        /**
         * Command to wait for Intake beam brake is tripped
         *
         * @param intake Intake Subsystem
         * @return Command
         */
        public static Command waitForIntake(Intake intake) {
            return Commands.waitUntil(() -> intake.getIndexerBeamBrakeStatus());
        }

    }
}
