package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

/** Auto which gets the 8, 7, and 6 note. */
public class Side876 extends ParallelCommandGroup {

    private double wristTarget = 0.0;

    private Command setWristTarget(double angleDegrees) {
        return Commands.runOnce(() -> {
            wristTarget = angleDegrees;
        });
    }

    /** Auto which gets the 8, 7, and 6 note. */
    public Side876(Swerve swerve, Shooter shooter, Intake intake, ElevatorWrist elevatorWrist) {
        addRequirements(swerve, shooter, intake, elevatorWrist);

        PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory("876p1");
        PathPlannerPath path2 = PathPlannerPath.fromChoreoTrajectory("876p2");
        PathPlannerPath path3 = PathPlannerPath.fromChoreoTrajectory("876p3");
        PathPlannerPath path4 = PathPlannerPath.fromChoreoTrajectory("876p4");

        Command followPath1 = AutoBuilder.followPath(path1);
        Command followPath2 = AutoBuilder.followPath(path2);
        Command followPath3 = AutoBuilder.followPath(path3);
        Command followPath4 = AutoBuilder.followPath(path4);

        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose());
            swerve.resetOdometry(initialState);
        });

        Command wristPositions = elevatorWrist.followPosition(
            () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
            () -> Rotation2d.fromDegrees(wristTarget));

        Command part1 = setWristTarget(20.0).andThen(followPath1)
            .andThen(intake.runIndexerMotor(1.0).withTimeout(0.5));
        Command part2 = followPath2.alongWith(intake.runIntakeMotor(1.0, 0.2))
            .andThen(setWristTarget(20.0).andThen(intake.runIndexerMotor(1.0).withTimeout(0.5)));
        Command part3 =
            setWristTarget(Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE.getDegrees())
                .andThen(followPath3.alongWith(intake.runIntakeMotor(1.0, 0.2))).andThen(
                    setWristTarget(20.0).andThen(intake.runIndexerMotor(1.0).withTimeout(0.5)));
        Command part4 =
            setWristTarget(Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE.getDegrees())
                .andThen(followPath4.alongWith(intake.runIntakeMotor(1.0, 0.2))).andThen(
                    setWristTarget(20.0).andThen(intake.runIndexerMotor(1.0).withTimeout(0.5)));
        addCommands(wristPositions.repeatedly(), shooter.shootSpeaker(),
            resetPosition.andThen(part1).andThen(part2).andThen(part3).andThen(part4));
    }



}
