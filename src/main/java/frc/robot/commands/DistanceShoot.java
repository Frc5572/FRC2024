package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class DistanceShoot extends SequentialCommandGroup {
    Shooter shooter;
    ElevatorWrist elevatorWrist;
    Swerve swerve;
    double angle = Math.tan(FieldConstants.Speaker.centerSpeakerOpening.getX());

    public DistanceShoot(Shooter shooter, ElevatorWrist elevatorWrist, Swerve swerve) {
        this.shooter = shooter;
        this.elevatorWrist = elevatorWrist;
        addRequirements(shooter);
        addCommands(null);
    }



}
