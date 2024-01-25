package frc.robot.subsystems.elevatorWrist;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Elevator and Wrist Subsystem
 */
public class ElevatorWrist implements Subsystem {
    public ElevatorWristIO io;
    public ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    public ElevatorWrist(ElevatorWristIO io) {
        this.io = io;
    }

}
