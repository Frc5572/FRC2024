package frc.robot.autos;

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
 * Resnick 2 Custom Auto
 */
public class Resnick2 extends SequentialCommandGroup {

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
    public Resnick2(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake,
        Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(swerveDrive);


        PathPlannerPath path1 = PathPlannerPath.fromPathFile("Resnick 2 Shoot Initial Note");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("Resnick 2 Intake P1");
        PathPlannerPath path3 = PathPlannerPath.fromPathFile("Resnick 2 Intake P2");
        PathPlannerPath path4 = PathPlannerPath.fromPathFile("Resnick 2 Intake P3");

        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);
        Command followPath4 = AutoBuilder.followPath(path4);


        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose());
            swerveDrive.resetOdometry(initialState);
        });
        SequentialCommandGroup part1 =
            followPath1.andThen(CommandFactory.shootSpeaker(shooter, intake).withTimeout(1.5));
        SequentialCommandGroup part2 = followPath2.deadlineWith(intake.runIntakeMotor(1, .2))
            .andThen(CommandFactory.shootSpeaker(shooter, intake).withTimeout(1.5));
        SequentialCommandGroup part3 = followPath3.deadlineWith(intake.runIntakeMotor(1, .2))
            .andThen(CommandFactory.shootSpeaker(shooter, intake).withTimeout(1.5));
        SequentialCommandGroup part4 = followPath4.deadlineWith(intake.runIntakeMotor(1, .2))
            .andThen(CommandFactory.shootSpeaker(shooter, intake).withTimeout(1.5));

        SequentialCommandGroup followPaths = part1.andThen(part2).andThen(part3).andThen(part4);

        Command autoAlignWrist = elevatorWrist.followPosition(
            () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            () -> elevatorWrist.getAngleFromDistance(swerveDrive.getPose()));

        addCommands(resetPosition, autoAlignWrist.alongWith(followPaths));
    }

}
