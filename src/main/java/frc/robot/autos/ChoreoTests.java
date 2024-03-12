package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;

public class ChoreoTests {

    private static double[] actionTimes = {0.8, // shoot
        1.0, // intake
        5.0, // shoot
        5.4, // intake
        8.55, // shoot
        8.9, // intake
        13.2 // shoot
    };

    private static Timer timer = new Timer();

    private static int[] parts = {6, 7};

    public static Command testPath(Swerve swerve, Intake intake, ElevatorWrist wrist) {
        PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory("SideStart");
        Command resetPosition = Commands.runOnce(() -> {
            Pose2d initialState =
                FieldConstants.allianceFlip(path1.getPreviewStartingHolonomicPose()
                    .plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180))));
            swerve.resetOdometry(initialState);
        });

        Command shootPreload = AutoBuilder.followPath(path1)
            .alongWith(wrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                Rotation2d.fromDegrees(35.0)).withTimeout(0.8))
            .andThen(intake.runIndexerMotor(1.0).withTimeout(0.5));

        Command fullSequence = shootPreload;
        String from = "Start";
        for (int i = 0; i < parts.length; i++) {
            PathPlannerPath path2 = PathPlannerPath.fromChoreoTrajectory(from + "To" + parts[i]);
            PathPlannerPath path3 = PathPlannerPath.fromChoreoTrajectory(parts[i] + "ToShoot");
            Command home =
                wrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE);
            fullSequence = fullSequence.andThen(
                AutoBuilder.followPath(path2).andThen(AutoBuilder.followPath(path3)).alongWith(home,
                    intake.runIntakeMotor(1.0, 0.2)),
                wrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    Rotation2d.fromDegrees(35.0)).withTimeout(0.5),
                intake.runIndexerMotor(1.0).withTimeout(0.3));
            from = "Shoot";
        }

        return resetPosition.andThen(new WaitCommand(0.01), fullSequence).finallyDo(() -> {
            intake.setIndexerMotor(0.0);
            intake.setIntakeMotor(0.0);
        });
    }

    private static boolean isShooting() {
        for (int i = 0; i < actionTimes.length; i++) {
            if (actionTimes[i] < timer.get()) {
                if (i % 2 == 0) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        return false;
    }

}
