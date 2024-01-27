package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberInputs {

    }

    public default void updateInputs(ClimberInputs inputs) {}

    public default void setClimberVoltage(double volts) {}
}
