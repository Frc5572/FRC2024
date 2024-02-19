package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevatorWrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

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
        // Supplier<Rotation2d> rotation = () -> new Rotation2d(Math
        // .atan(Constants.ShooterConstants.HEIGHT_FROM_SPEAKER / swerve.distanceFromSpeaker()));
        Command runIndexer = intake.runIndexerMotor(Constants.IntakeConstants.INDEX_MOTOR_FORWARD);
        // Command moveElevatorWrist = elevatorWrist
        // .followPosition(() -> Constants.ShooterConstants.HEIGHT_FROM_LOWEST_POS, rotation);
        Command runshooter = shooter.shootWithDistance(() -> 0);
        Command readytoShoot = Commands.waitUntil(() -> shooter.atSetpoint());
        return runshooter.alongWith(readytoShoot.andThen(runIndexer)).finallyDo(x -> {
            shooter.setBottomMotor(0);
            shooter.setTopMotor(0);
            intake.setIndexerMotor(0);
        });
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
