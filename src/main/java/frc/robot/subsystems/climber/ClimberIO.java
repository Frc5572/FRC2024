package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/**
 * Susbsytem for climber IO
 */
public interface ClimberIO {
    @AutoLog
    public static class ClimberInputs {
        public double climberLeftMotorVoltage;
        public double climberLeftMotorAmp;
        public double climberRightMotorVoltage;
        public double climberRightMotorAmp;

    }

    public default void updateInputs(ClimberInputs inputs) {}

    public default void setClimberVoltage(double volts) {}
}
