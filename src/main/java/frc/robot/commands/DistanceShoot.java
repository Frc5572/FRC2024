package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.shooter.Shooter;

public class DistanceShoot extends Command {
    Shooter shooter;
    ElevatorWrist elevatorWrist;

    public DistanceShoot(Shooter shooter, ElevatorWrist elevatorWrist) {
        this.shooter = shooter;
        this.elevatorWrist = elevatorWrist;
    }

}
