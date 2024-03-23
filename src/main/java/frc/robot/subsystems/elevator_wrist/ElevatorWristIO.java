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
        public double leftElevatorMotorSupplyVoltage;
        public double leftElevatorRelativeEncRawValue;
        public double leftElevatorMotorVoltage;
        public double leftElevatorMotorAmp;
        public double leftElevatorMotorTemp;
        public double rightElevatorMotorSupplyVoltage;
        public double rightElevatorRelativeEncRawValue;
        public double rightElevatorMotorVoltage;
        public double rightElevatorMotorAmp;
        public double rightElevatorMotorTemp;
        public double wristMotorVoltage;
        public double wristMotorAmp;
        public double wristMotorTemp;
    }

    public default void updateInputs(ElevatorWristInputs inputs) {}

    public default void setElevatorVoltage(double voltage) {}

    public default void setWristVoltage(double voltage) {}

}
