package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
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
     * @param elevatorWrist Elevator and Wrist subsystem
     * @param swerve Swerve subsystem
     * @param intake Intake subsystem
     * @return Returns a command
     */
    public static Command shootSpeaker(Shooter shooter, ElevatorWrist elevatorWrist, Swerve swerve,
        Intake intake) {
        Supplier<Rotation2d> rotation = () -> new Rotation2d(Math
            .atan(Constants.ShooterConstants.HEIGHT_FROM_SPEAKER / swerve.distanceFromSpeaker()));
        Command runIndexer = intake.runIndexerMotor(Constants.IntakeConstants.INDEX_MOTOR_FORWARD);
        Command moveElevatorWrist = elevatorWrist
            .followPosition(() -> Constants.ShooterConstants.HEIGHT_FROM_LOWEST_POS, rotation);
        Command runshooter = shooter.shootWithDistance(() -> swerve.distanceFromSpeaker());
        Command readytoShoot =
            Commands.waitUntil(() -> elevatorWrist.atGoal() && shooter.atSetpoint());
        return runshooter.alongWith(moveElevatorWrist).alongWith(readytoShoot.andThen(runIndexer));
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
}
