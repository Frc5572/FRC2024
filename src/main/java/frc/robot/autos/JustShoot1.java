package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.math.StateEstimator;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/**
 * P32 Auto
 */
public class JustShoot1 extends SequentialCommandGroup {

    Swerve swerveDrive;
    ElevatorWrist elevatorWrist;
    Intake intake;
    Shooter shooter;

    /**
     * P32 Auto
     *
     * @param swerveDrive Swerve Drive Subsystem
     * @param elevatorWrist Elevator Wrist Subsystem
     * @param intake Intake Subsystem
     * @param shooter Shooter Subsystem
     */
    public JustShoot1(Swerve swerveDrive, StateEstimator estimator, ElevatorWrist elevatorWrist,
        Intake intake, Shooter shooter) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;

        PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory("just shoot 1");

        Command wait = Commands.waitSeconds(9);
        Command followPath1 = AutoBuilder.followPath(path1);

        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose()
                    .plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))));
            swerveDrive.resetOdometry(initialState);
        });
        SequentialCommandGroup part1 = followPath1.andThen(CommandFactory.Auto.runIndexer(intake));

        SequentialCommandGroup followPaths = part1;

        Command shootCommand = shooter.shootSpeaker();
        Command autoTarget = elevatorWrist.followPosition(
            () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            () -> elevatorWrist.getAngleFromDistance(estimator.getPoseEstimate()));

        addCommands(resetPosition, wait, followPaths.alongWith(shootCommand, autoTarget));
    }
}
