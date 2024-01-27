package frc.robot.subsystems.elevator_wrist;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorWristIO {
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
