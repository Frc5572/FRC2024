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
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Resnick 2 Custom Auto
 */
public class Resnick3 extends SequentialCommandGroup {

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
    public Resnick3(Swerve swerveDrive, ElevatorWrist elevatorWrist, Intake intake,
        Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(swerveDrive);

        Supplier<Integer> numNotes = () -> RobotContainer.numNoteChooser.getSelected();

        PathPlannerPath path0 = PathPlannerPath.fromPathFile("1 - Resnick 2 Shoot Initial Note");
        PathPlannerPath path1 = PathPlannerPath.fromPathFile("2 - Resnick 2 Intake P1");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("3 - Resnick 2 Intake P2");
        PathPlannerPath path3 = PathPlannerPath.fromPathFile("4 - Resnick 2 Intake P3");
        PathPlannerPath path4 = PathPlannerPath.fromPathFile("1 - Resnick 3 Intake P4");
        PathPlannerPath path5 = PathPlannerPath.fromPathFile("2 - Resnick 3 Intake P5");
        PathPlannerPath path6 = PathPlannerPath.fromPathFile("3 - Resnick 3 Intake P6");

        Command followPath0 = AutoBuilder.followPath(path0);
        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);
        Command followPath4 = AutoBuilder.followPath(path4);
        Command followPath5 = AutoBuilder.followPath(path5);
        Command followPath6 = AutoBuilder.followPath(path6);


        // Supplier<Command> readytoShoot =
        // () -> Commands.waitUntil(() -> shooter.readyToShoot() && elevatorWrist.atGoal());
        Command runshooter = shooter.shootSpeaker();
        Supplier<Command> shootNote =
            () -> Commands.waitUntil(() -> shooter.readyToShoot() && elevatorWrist.atGoal())
                .andThen(intake.runIndexerMotor(1).withTimeout(.7));

        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path0.getPreviewStartingHolonomicPose());
            swerveDrive.resetOdometry(initialState);
        });

        SequentialCommandGroup part0 = followPath0.andThen(shootNote.get());
        SequentialCommandGroup part1 = followPath1.andThen(shootNote.get());
        SequentialCommandGroup part2 = followPath2.andThen(shootNote.get());
        SequentialCommandGroup part3 = followPath3.andThen(shootNote.get());
        SequentialCommandGroup part4 = followPath4.andThen(shootNote.get());
        SequentialCommandGroup part5 = followPath5.andThen(shootNote.get());
        SequentialCommandGroup part6 = followPath6.andThen(shootNote.get());

        Command runPart1 = Commands.either(part1, Commands.none(), () -> numNotes.get() > 0);
        Command runPart2 = Commands.either(part2, Commands.none(), () -> numNotes.get() > 1);
        Command runPart3 = Commands.either(part3, Commands.none(), () -> numNotes.get() > 2);
        Command runPart4 = Commands.either(part4, Commands.none(), () -> numNotes.get() > 3);
        Command runPart5 = Commands.either(part5, Commands.none(), () -> numNotes.get() > 4);
        Command runPart6 = Commands.either(part6, Commands.none(), () -> numNotes.get() > 5);

        SequentialCommandGroup followPaths = part0.andThen(runPart1).andThen(runPart2)
            .andThen(runPart3).andThen(runPart4).andThen(runPart5).andThen(runPart6);

        Command autoAlignWrist = elevatorWrist.followPosition(
            () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            () -> elevatorWrist.getAngleFromDistance(swerveDrive.getPose()));

        addCommands(resetPosition, followPaths.alongWith(autoAlignWrist, runshooter));
    }

}
