package frc.robot.subsystems.elevator_wrist;

import org.littletonrobotics.junction.AutoLog;

/**
 * Elevator and wrist IO class
 */
public interface ElevatorWristIO {
    /**
     * Elevator and wrist inputs
     */
    @AutoLog

    public static class ElevatorWristInputs {
        public double elevatorRelativeEncRawValue;
        public boolean topLimitSwitch;
        public boolean bottomLimitSwitch;

        public double wristAbsoluteEncRawValue;
    }

    public default void updateInputs(ElevatorWristInputs inputs) {}

    public default void setElevatorVoltage(double voltage) {}

    public default void setWristVoltage(double voltage) {}

}
