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
import frc.robot.RobotContainer;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * P8765 Auto to shoot or dump
 */
public class P8765 extends SequentialCommandGroup {

    Swerve swerveDrive;
    ElevatorWrist elevatorWrist;
    Intake intake;
    Shooter shooter;

    /**
     * P8765 Auto to shoot or dump
     *
     * @param swerveDrive Swerve Drive Subsystem
     * @param elevatorWrist Elevator Wrist Subsystem
     * @param intake Intake Subsystem
     * @param shooter Shooter Subsystem
     */
    public P8765(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake, Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;

        PathPlannerPath path0 = PathPlannerPath.fromChoreoTrajectory("P8765-initial");
        PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory("P8765-p8");
        PathPlannerPath path2 = PathPlannerPath.fromChoreoTrajectory("P8765-p7");
        PathPlannerPath path3 = PathPlannerPath.fromChoreoTrajectory("P8765-p6");
        PathPlannerPath path1_dump = PathPlannerPath.fromChoreoTrajectory("P8765-p8-dump");
        PathPlannerPath path2_dump = PathPlannerPath.fromChoreoTrajectory("P8765-p7-dump");
        PathPlannerPath path3_dump = PathPlannerPath.fromChoreoTrajectory("P8765-p6-dump");
        PathPlannerPath path4_dump = PathPlannerPath.fromChoreoTrajectory("P8765-p5-dump");
        PathPlannerPath path5_dump = PathPlannerPath.fromChoreoTrajectory("P8765-p4-dump");

        // Shoot Paths
        Command followPath0 = AutoBuilder.followPath(path0);
        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);

        // Dump Paths
        Command followPath1_dump = AutoBuilder.followPath(path1_dump);
        Command followPath2_dump = AutoBuilder.followPath(path2_dump);
        Command followPath3_dump = AutoBuilder.followPath(path3_dump);
        Command followPath4_dump = AutoBuilder.followPath(path4_dump);
        Command followPath5_dump = AutoBuilder.followPath(path5_dump);

        Command part0 = followPath0
            .alongWith(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(38.5)).withTimeout(1))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part1 = followPath1.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(32.5)).withTimeout(1.3))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part2 = followPath2.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(32.5)).withTimeout(1.3))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part3 = followPath3.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(32.5)).withTimeout(1.3))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));


        Command part1_dump = followPath1_dump.alongWith(CommandFactory.intakeNote(intake))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part2_dump = followPath2_dump.alongWith(CommandFactory.intakeNote(intake))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part3_dump = followPath3_dump.alongWith(CommandFactory.intakeNote(intake))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part4_dump = followPath4_dump.alongWith(CommandFactory.intakeNote(intake))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part5_dump = followPath5_dump.alongWith(CommandFactory.intakeNote(intake));

        Command wait = Commands.waitSeconds(.1);
        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path0.getPreviewStartingHolonomicPose()
                    .plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))));
            swerveDrive.resetOdometry(initialState);
        });

        Command followPaths = part0.andThen(Commands.either(
            // Run Dump Paths
            Commands.sequence(part1_dump, part2_dump, part3_dump, part4_dump, part5_dump),
            // Run Shooting paths
            Commands.sequence(part1, part2, part3),
            () -> RobotContainer.dumpNotes.getEntry().getBoolean(false)));
        Command shootCommand = shooter.shootSpeaker();

        addCommands(resetPosition, wait, followPaths.deadlineWith(shootCommand));
    }
}
