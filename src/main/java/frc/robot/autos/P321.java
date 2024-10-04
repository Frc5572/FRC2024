package frc.robot.autos;

import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * P321 Auto
 */
public class P321 extends SequentialCommandGroup {

    Swerve swerveDrive;
    ElevatorWrist elevatorWrist;
    Intake intake;
    Indexer indexer;
    Shooter shooter;

    /**
     * P321 Auto
     *
     * @param swerveDrive Swerve Drive Subsystem
     * @param elevatorWrist Elevator Wrist Subsystem
     * @param intake Intake Subsystem
     * @param indexer Indexer SUbsystem
     * @param shooter Shooter Subsystem
     */
    public P321(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake, Indexer indexer,
        Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;

        PathPlannerPath path1 = PathPlannerPath.fromPathFile("1 - Resnick 2 Shoot Initial Note");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("2 - Resnick 2 Intake P3");
        PathPlannerPath path3 = PathPlannerPath.fromPathFile("3 - Resnick 2 Intake P2");
        PathPlannerPath path4 = PathPlannerPath.fromPathFile("4 - Resnick 2 Intake P1");
        PathPlannerPath path5 = PathPlannerPath.fromPathFile("5 - Resnick 2 midline");
        PathPlannerPath path6 = PathPlannerPath.fromPathFile("6 - Resnick 2 midline 2");

        Command wait = Commands.waitSeconds(.01);
        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);
        Command followPath4 = AutoBuilder.followPath(path4);
        Command followPath5 = AutoBuilder.followPath(path5);
        Command followPath6 = AutoBuilder.followPath(path6);

        BooleanSupplier abort = indexer.noteInIndexer.negate().and(intake.noteInIntake.negate());
        BooleanSupplier goToCenter = () -> RobotContainer.goToCenter.getEntry().getBoolean(false);

        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose());
            swerveDrive.resetOdometry(initialState);
        });

        double elevatorHeight = 28.1;
        SequentialCommandGroup part1 = followPath1
            .alongWith(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(39.0)).withTimeout(1.0))
            .andThen(CommandFactory.Auto.runIndexer(indexer).asProxy());
        SequentialCommandGroup part2 = followPath2.alongWith(elevatorWrist
            .goToPosition(elevatorHeight, Rotation2d.fromDegrees(37.0)).withTimeout(.5))
            .andThen(CommandFactory.Auto.runIndexer(indexer).asProxy());
        SequentialCommandGroup part3 = followPath3.alongWith(elevatorWrist
            .goToPosition(elevatorHeight, Rotation2d.fromDegrees(37.5)).withTimeout(.5))
            .andThen(CommandFactory.Auto.runIndexer(indexer).asProxy());
        SequentialCommandGroup part4 = followPath4
            .alongWith(elevatorWrist.goToPosition(elevatorHeight, Rotation2d.fromDegrees(36.0))
                .withTimeout(.5))
            .andThen(CommandFactory.Auto.runIndexer(indexer).asProxy())
            .andThen(elevatorWrist.homePosition().withTimeout(.5));
        Command part5 = followPath5.andThen(Commands.either(Commands.none(),
            Commands.sequence(Commands.waitUntil(indexer.noteInIndexer),
                CommandFactory.Auto.runIndexer(indexer).asProxy()),
            abort));
        Command part6 = followPath6.andThen(Commands.either(Commands.none(),
            Commands.sequence(Commands.waitUntil(indexer.noteInIndexer),
                CommandFactory.Auto.runIndexer(indexer).asProxy()),
            abort));
        Command midline =
            Commands.either(Commands.sequence(part5, part6), Commands.none(), goToCenter);

        Command followPaths = Commands.sequence(part1, part2, part3, part4, midline);

        // Command autoAlignWrist = CommandFactory.autoAngleWristSpeaker(elevatorWrist,
        // swerveDrive);
        Command shootCommand = shooter.shootSpeaker();

        addCommands(resetPosition, wait, followPaths.alongWith(shootCommand));
    }
}
