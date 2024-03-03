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

        PathPlannerPath path1 = PathPlannerPath.fromPathFile("1 - Resnick 4 Intake P2");
        PathPlannerPath path2 = PathPlannerPath.fromPathFile("2 - Resnick 4 Intake P2");

        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);


        // Supplier<Command> readytoShoot =
        // () -> Commands.waitUntil(() -> shooter.readyToShoot() && elevatorWrist.atGoal());
        Command runshooter = shooter.shootSpeaker();
        Supplier<Command> shootNote =
            () -> Commands.waitUntil(() -> shooter.readyToShoot() && elevatorWrist.atGoal())
                .andThen(intake.runIndexerMotor(1).withTimeout(.7));

        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose());
            swerveDrive.resetOdometry(initialState);
        });

        Command part0 = shootNote.get();
        SequentialCommandGroup part1 = followPath1.andThen(shootNote.get());
        SequentialCommandGroup part2 = followPath2.andThen(shootNote.get());

        Command runPart1 = Commands.either(part1, Commands.none(), () -> numNotes.get() > 0);
        Command runPart2 = Commands.either(part2, Commands.none(), () -> numNotes.get() > 1);

        SequentialCommandGroup followPaths = part0.andThen(runPart1).andThen(runPart2);

        Command autoAlignWrist = elevatorWrist.followPosition(
            () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            () -> elevatorWrist.getAngleFromDistance(swerveDrive.getPose()));

        addCommands(resetPosition, followPaths.alongWith(autoAlignWrist, runshooter));
    }

}
