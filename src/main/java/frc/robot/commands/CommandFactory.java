package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
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
        BooleanSupplier sensor = () -> intake.getSensorStatus();
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
        return runshooter.alongWith(readytoShoot.withTimeout(2).andThen(runIndexer));
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
     * Command to climb robot, then set up position to score.
     *
     * @param climber Climber subsystem
     * @param elevatorWrist Elevator and Wrist subsystem
     * @return Returns auto climb command
     */
    public static Command autoClimb(Climber climber, ElevatorWrist elevatorWrist) {
        Command initialExtension =
            elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.CLIMBING_HEIGHT,
                Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE);
        Command hooksAttach =
            elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE);
        Command climb = climber.getToPosition(Constants.ClimberConstants.CLIMBING_DISTANCE);
        Command extendToTrap =
            elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.TRAP_HEIGHT,
                Constants.ElevatorWristConstants.SetPoints.TRAP_ANGLE);
        return initialExtension.andThen(hooksAttach).andThen(climb).andThen(extendToTrap);
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
            return Commands.waitUntil(() -> intake.getSensorStatus())
                .andThen(Commands.waitSeconds(.25)).deadlineWith(intake.runIndexerMotor(1));
        }

        /**
         * Command to wait for Intake beam brake is tripped
         *
         * @param intake Intake Subsystem
         * @return Command
         */
        public static Command waitForIntake(Intake intake) {
            return Commands.waitUntil(() -> !intake.getSensorStatus());
        }

    }
}
