package frc.robot.autos;

import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.StateEstimator;
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
    public P8765(Swerve swerveDrive, StateEstimator estimator, ElevatorWrist elevatorWrist,
        Intake intake, Shooter shooter) {
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



        Trigger noteInIndexer = new Trigger(() -> this.intake.getIndexerBeamBrakeStatus())
            .debounce(0.25, Debouncer.DebounceType.kRising);
        Trigger noteInIntake = new Trigger(() -> this.intake.getintakeBeamBrakeStatus())
            .debounce(0.25, Debouncer.DebounceType.kRising);
        BooleanSupplier abort = () -> !noteInIndexer.getAsBoolean() && !noteInIntake.getAsBoolean();
        BooleanSupplier dumpOrNot = () -> RobotContainer.dumpNotes.getEntry().getBoolean(false);
        double elevatorHeight = 28.0;
        Command part0 = followPath0
            .alongWith(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(31.0)).withTimeout(1.5))
            .andThen(Commands.waitSeconds(.1)).andThen(CommandFactory.Auto.runIndexer(intake));
        // .andThen(Commands.either(elevatorWrist.homePosition().withTimeout(.5), Commands.none(),
        // dumpOrNot));
        Command part1 =
            followPath1
                .deadlineWith(CommandFactory.intakeNote(intake),
                    elevatorWrist.homePosition().withTimeout(1.0))
                .andThen(Commands.either(Commands.none(),
                    Commands.sequence(CommandFactory.intakeNote(intake)
                        .alongWith(elevatorWrist
                            .followPosition(
                                () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                                () -> elevatorWrist
                                    .getAngleFromDistance(estimator.getPoseEstimate()))
                            .withTimeout(1.5)),
                        CommandFactory.Auto.runIndexer(intake)),
                    abort));
        Command part2 =
            followPath2
                .deadlineWith(CommandFactory.intakeNote(intake),
                    elevatorWrist.homePosition().withTimeout(1.0))
                .andThen(Commands.either(Commands.none(),
                    Commands.sequence(CommandFactory.intakeNote(intake)
                        .alongWith(elevatorWrist
                            .followPosition(
                                () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                                () -> elevatorWrist
                                    .getAngleFromDistance(estimator.getPoseEstimate()))
                            .withTimeout(1.5)),
                        CommandFactory.Auto.runIndexer(intake)),
                    abort));
        Command part3 = followPath3.alongWith(CommandFactory.intakeNote(intake))
            .andThen(
                elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(32.5)).withTimeout(1.5))
            .andThen(CommandFactory.Auto.runIndexer(intake))
            .andThen(elevatorWrist.homePosition().withTimeout(.5));


        Command part1_dump = followPath1_dump.deadlineWith(CommandFactory.intakeNote(intake))
            .andThen(Commands.either(Commands.none(),
                Commands.sequence(CommandFactory.intakeNote(intake),
                    CommandFactory.Auto.runIndexer(intake)),
                abort))
            .andThen(elevatorWrist.homePosition().withTimeout(.1));
        Command part2_dump = followPath2_dump.deadlineWith(CommandFactory.intakeNote(intake))
            .andThen(Commands.either(Commands.none(),
                Commands.sequence(CommandFactory.intakeNote(intake),
                    CommandFactory.Auto.runIndexer(intake)),
                abort))
            .andThen(elevatorWrist.homePosition().withTimeout(.1));
        Command part3_dump = followPath3_dump.deadlineWith(CommandFactory.intakeNote(intake))
            .andThen(Commands.either(Commands.none(),
                Commands.sequence(CommandFactory.intakeNote(intake),
                    CommandFactory.Auto.runIndexer(intake)),
                abort))
            .andThen(elevatorWrist.homePosition().withTimeout(.1));
        Command part4_dump = followPath4_dump.alongWith(CommandFactory.intakeNote(intake));

        Command wait = Commands.waitSeconds(.01);
        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path0.getPreviewStartingHolonomicPose()
                    .plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))));
            swerveDrive.resetOdometry(initialState);
        });

        Command followPaths = part0.andThen(Commands.either(
            // Run Dump Paths
            Commands.sequence(part1_dump, part2_dump, part3_dump, part4_dump),
            // Run Shooting paths
            Commands.sequence(part1, part2), dumpOrNot));
        Command shootCommand = shooter.shootSpeaker();

        addCommands(resetPosition, wait, followPaths.deadlineWith(shootCommand));
    }
}
