package frc.robot.commands;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class ShootFromSpeaker extends SequentialCommandGroup {

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
    public ShootFromSpeaker(Swerve swerveDrive, Shooter shooter, ElevatorWrist elevatorWrist,
        Intake intake) {
        this.swerveDrive = swerveDrive;
        this.elevatorWrist = elevatorWrist;
        this.intake = intake;
        this.shooter = shooter;

        Pose2d goToPose = new Pose2d(FieldConstants.startingLineX + Units.inchesToMeters(12),
            FieldConstants.Speaker.centerSpeakerOpening.getY(), Rotation2d.fromDegrees(180));
        Command moveToShoot = new MoveToPos(swerveDrive, () -> goToPose, true);

        Command moveWrist = elevatorWrist.goToPosition(
            Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT, Rotation2d.fromDegrees(50));

        addCommands(
            moveToShoot.alongWith(Commands.waitUntil(() -> checkLocation()).andThen(moveWrist)),
            CommandFactory.shootSpeaker(shooter, intake));
    }

    private boolean checkLocation() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Blue
                && swerveDrive.getPose().getX() < FieldConstants.StagingLocations.spikeX)
                || (alliance.get() == DriverStation.Alliance.Red && swerveDrive.getPose()
                    .getX() > FieldConstants.fieldWidth - FieldConstants.StagingLocations.spikeX);
        }
        return false;
    }
}
