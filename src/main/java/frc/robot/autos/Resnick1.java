package frc.robot.autos;

import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
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
 * Resnick 1 Custom Auto
 */
public class Resnick1 extends SequentialCommandGroup {

    Swerve swerveDrive;
    ElevatorWrist elevatorWrist;
    Intake intake;
    Shooter shooter;

    /**
     * Resnick 1 Custom Auto
     *
     * @param swerveDrive Swerve Drive Subsystem
     * @param elevatorWrist Elevator Wrist Subsystem
     * @param intake Intake Subsystem
     * @param shooter Shooter Subsystem
     */
    public Resnick1(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake,
        Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(swerveDrive);


        PathPlannerPath path1 = PathPlannerPath.fromPathFile("1 - Resnick 1 Shoot Initial Note");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("2 - Resnick 1 Intake P1");
        PathPlannerPath path3 = PathPlannerPath.fromPathFile("3 - Resnick 1 Intake P2");
        PathPlannerPath path4 = PathPlannerPath.fromPathFile("4 - Resnick 1 Intake P3");

        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);
        Command followPath4 = AutoBuilder.followPath(path4);

        Supplier<Command> runIndexer = () -> Commands.waitUntil(() -> intake.getSensorStatus())
            .andThen(Commands.waitSeconds(.25)).deadlineWith(intake.runIndexerMotor(1));
        Supplier<Command> waitForIntake = () -> Commands.waitUntil(() -> !intake.getSensorStatus());


        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose());
            swerveDrive.resetOdometry(initialState);
        });
        SequentialCommandGroup part1 = followPath1.andThen(runIndexer.get());
        SequentialCommandGroup part2 = followPath2.alongWith(CommandFactory.intakeNote(intake))
            .andThen(waitForIntake.get()).andThen(runIndexer.get());
        SequentialCommandGroup part3 = followPath3.alongWith(CommandFactory.intakeNote(intake))
            .andThen(waitForIntake.get()).andThen(runIndexer.get());
        SequentialCommandGroup part4 = followPath4.alongWith(CommandFactory.intakeNote(intake))
            .andThen(waitForIntake.get()).andThen(runIndexer.get());

        SequentialCommandGroup followPaths = part1.andThen(part2).andThen(part3).andThen(part4);

        Command autoAlignWrist = elevatorWrist.followPosition(
            () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            () -> elevatorWrist.getAngleFromDistance(swerveDrive.getPose()));

        Command shootCommand = shooter.shootSpeaker();

        addCommands(resetPosition, followPaths.alongWith(autoAlignWrist, shootCommand));
    }



}
