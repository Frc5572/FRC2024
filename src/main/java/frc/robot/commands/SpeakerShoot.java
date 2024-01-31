package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class SpeakerShoot extends SequentialCommandGroup {
    Shooter shooter;
    ElevatorWrist elevatorWrist;
    Swerve swerve;
    DoubleSupplier angle;

    public SpeakerShoot(Shooter shooter, ElevatorWrist elevatorWrist, Swerve swerve) {
        this.shooter = shooter;
        this.elevatorWrist = elevatorWrist;
        this.swerve = swerve;
        angle = () -> Math.tan(Constants.ShooterConstants.HEIGHT_FROM_SPEAKER
            / Constants.ShooterConstants.DISTANCE_FROM_SPEAKER(swerve::getPose));
        addRequirements(shooter);
        addCommands(shooter.shootWithDistance(angle));
    }



}
