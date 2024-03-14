package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Resnick 2 Custom Auto
 */
public class P675 extends SequentialCommandGroup {

    Swerve swerveDrive;
    ElevatorWrist elevatorWrist;
    Intake intake;
    Shooter shooter;

    /**
     * Resnick 2 Custom Auto
     *
     * @param swerveDrive Swerve Drive Subsystem
     * @param elevatorWrist Elevator Wrist Subsystem
     * @param intake Intake Subsystem
     * @param shooter Shooter Subsystem
     */
    public P675(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake, Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;

        PathPlannerPath path0 = PathPlannerPath.fromChoreoTrajectory("p675-initial");
        PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory("p675-p6");
        PathPlannerPath path2 = PathPlannerPath.fromChoreoTrajectory("p675-p7");
        PathPlannerPath path3 = PathPlannerPath.fromChoreoTrajectory("p675-p5");

        Command wait = Commands.waitSeconds(.01);
        Command followPath0 = AutoBuilder.followPath(path0);
        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);

        Command part0 = followPath0.andThen(CommandFactory.Auto.runIndexer(intake))
            .alongWith(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(35.0)).withTimeout(1))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part1 = followPath1.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(31.5)).withTimeout(1))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part2 = followPath2.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(31.5)).withTimeout(1))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part3 = followPath3.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(31.5)).withTimeout(1))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));


        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path0.getPreviewStartingHolonomicPose()
                    .plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))));
            swerveDrive.resetOdometry(initialState);
        });

        Command followPaths = part0.andThen(part1).andThen(part2).andThen(part3);
        Command shootCommand = shooter.shootSpeaker();

        addCommands(resetPosition, wait, followPaths.deadlineWith(shootCommand));
    }
}
