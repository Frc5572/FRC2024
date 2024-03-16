package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
public class P3675 extends SequentialCommandGroup {

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
    public P3675(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake, Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;

        PathPlannerPath path1 = PathPlannerPath.fromPathFile("1 - Resnick 2 Shoot Initial Note");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("2 - Resnick 2 Intake P3");
        PathPlannerPath path3 = PathPlannerPath.fromChoreoTrajectory("p3675-p6");
        PathPlannerPath path4 = PathPlannerPath.fromChoreoTrajectory("p675-p7");
        PathPlannerPath path5 = PathPlannerPath.fromChoreoTrajectory("p675-p5");

        // Command wait = Commands.waitSeconds(1);
        // Command followPath0 = AutoBuilder.followPath(path0);
        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);
        Command followPath4 = AutoBuilder.followPath(path4);
        Command followPath5 = AutoBuilder.followPath(path5);


        Command wait = Commands.waitSeconds(.1);
        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose());
            swerveDrive.resetOdometry(initialState);
        });
        SequentialCommandGroup part1 = followPath1
            .alongWith(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(39.0)).withTimeout(1))
            .andThen(CommandFactory.Auto.runIndexer(intake));

        SequentialCommandGroup part2 = followPath2
            .alongWith(CommandFactory.intakeNote(intake),
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(37.0)).withTimeout(.5))
            .andThen(CommandFactory.Auto.runIndexer(intake));

        Command part3 = followPath3.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(32.5)).withTimeout(1.3))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));

        Command part4 = followPath4.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(32.5)).withTimeout(1.3))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));

        Command part5 = followPath5.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(32.5)).withTimeout(1.3))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));

        SequentialCommandGroup followPaths =
            part1.andThen(part2).andThen(part3).andThen(part4).andThen(part5);
        Command shootCommand = shooter.shootSpeaker();

        addCommands(resetPosition, wait, followPaths.deadlineWith(shootCommand));
    }
}
