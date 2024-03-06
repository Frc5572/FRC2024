package frc.robot.autos;

import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Resnick 2 Custom Auto
 */
public class Resnick4 extends SequentialCommandGroup {

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
    public Resnick4(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake,
        Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(swerveDrive);

        Supplier<Integer> numNotes = () -> RobotContainer.numNoteChooser.getSelected();

        PathPlannerPath path1 = PathPlannerPath.fromPathFile("1 - Resnick 4 Shoot Initial");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("2 - Resnick 4 Intake P2");
        PathPlannerPath path3 = PathPlannerPath.fromPathFile("3 - Resnick 4 Intake P1");

        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);

        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose());
            swerveDrive.resetOdometry(initialState);
        });


        Command autoAlignWrist = CommandFactory.autoAngleWristSpeaker(elevatorWrist, swerveDrive);
        Command shootCommand = shooter.shootSpeaker();

        SequentialCommandGroup part1 = followPath1.andThen(Commands.waitSeconds(1))
            .andThen(CommandFactory.Auto.runIndexer(intake));

        SequentialCommandGroup part2 = followPath2.alongWith(CommandFactory.intakeNote(intake))
            .andThen(CommandFactory.Auto.runIndexer(intake));

        SequentialCommandGroup followPaths = part1.andThen(part2).andThen(Commands.waitSeconds(5));
        addCommands(resetPosition, followPaths.alongWith(autoAlignWrist, shootCommand));
    }

}
