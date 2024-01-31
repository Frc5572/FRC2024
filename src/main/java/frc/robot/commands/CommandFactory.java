package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        BooleanSupplier sensor = () -> intake.getSensorStatus();
        Command moveElevatorWrist =
            elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE);
        Command runIntakeIndexer =
            intake.runIntakeMotor(Constants.IntakeConstants.INDEX_MOTOR_FORWARD);
        return moveElevatorWrist.andThen(runIntakeIndexer).unless(sensor);
    }

    /**
     * Wrist follows the speaker until it is met, then it shoots
     *
     * @param shooter Shooter subsystem
     * @param elevatorWrist Elevator and Wrist subsystem
     * @param swerve Swerve subsystem
     * @return Returns a command
     */
    public static Command shootSpeaker(Shooter shooter, ElevatorWrist elevatorWrist,
        Swerve swerve) {
        double angle = Math.atan2(Constants.ShooterConstants.HEIGHT_FROM_SPEAKER,
            swerve.distanceFromSpeaker(swerve::getPose));
        Command moveElevatorWrist =
            elevatorWrist.followPosition(() -> Constants.ShooterConstants.HEIGHT_FROM_LOWEST_POS,
                () -> Rotation2d.fromRadians(angle));
        Command shoot =
            shooter.shootWithDistance(() -> swerve.distanceFromSpeaker(swerve::getPose));
        Command waitForElevator = Commands.waitUntil(() -> elevatorWrist.atGoal());
        return moveElevatorWrist.alongWith(waitForElevator.andThen(shoot));
    }
}


