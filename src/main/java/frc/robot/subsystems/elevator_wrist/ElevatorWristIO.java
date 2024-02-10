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
        public boolean topLimitSwitch;
        public boolean bottomLimitSwitch;
        public double wristAbsoluteEncRawValue;
        public double elevatorMotorSupplyVoltage;
        public double elevatorRelativeEncRawValue;
        public double elevatorMotorMotorVoltage;
        public double elevatorMotorAmp;
        public double wristMotorVoltage;
        public double wristMotorAmp;
    }

    public default void updateInputs(ElevatorWristInputs inputs) {}

    public default void setElevatorVoltage(double voltage) {}

    public default void setWristVoltage(double voltage) {}

}
