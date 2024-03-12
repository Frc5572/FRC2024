package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Resnick 1 Custom Auto
 */
public class P123Choreo extends SequentialCommandGroup {

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
    public P123Choreo(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake,
        Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;
        // addRequirements(swerveDrive);
        // SmartDashboard.putBoolean("Auto Status", false);


        PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory("P123 - initial");
        PathPlannerPath path2 = PathPlannerPath.fromChoreoTrajectory("P123 - p1");
        PathPlannerPath path3 = PathPlannerPath.fromChoreoTrajectory("P123 - p2");
        PathPlannerPath path4 = PathPlannerPath.fromChoreoTrajectory("P123 - p3");

        Command wait = Commands.waitSeconds(0.01);
        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);
        Command followPath4 = AutoBuilder.followPath(path4);

        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose());
            swerveDrive.resetOdometry(initialState);
        });
        SequentialCommandGroup part1 = followPath1
            // .andThen(Commands.runOnce(() -> SmartDashboard.putBoolean("Auto Status", true)))
            .andThen(CommandFactory.Auto.runIndexer(intake));
        SequentialCommandGroup part2 = followPath2.alongWith(CommandFactory.intakeNote(intake))
            .andThen(CommandFactory.Auto.runIndexer(intake));
        SequentialCommandGroup part3 = followPath3.alongWith(CommandFactory.intakeNote(intake))
            .andThen(CommandFactory.Auto.runIndexer(intake));
        SequentialCommandGroup part4 = followPath4.alongWith(CommandFactory.intakeNote(intake))
            .andThen(CommandFactory.Auto.runIndexer(intake));

        SequentialCommandGroup followPaths = part1.andThen(part2).andThen(part3).andThen(part4);

        Command autoAlignWrist = CommandFactory.autoAngleWristSpeaker(elevatorWrist, swerveDrive);
        Command shootCommand = shooter.shootSpeaker();

        addCommands(resetPosition, wait, followPaths.alongWith(autoAlignWrist, shootCommand));
    }
}
