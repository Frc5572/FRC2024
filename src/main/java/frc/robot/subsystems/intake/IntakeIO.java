package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Intake IO Interface
 */
public interface IntakeIO {

    /**
     * Intake Inputs to Log
     */
    @AutoLog
    public static class IntakeInputs {
        public double intakeSupplyVoltage;
        public double indexerSupplyVoltage;
        public double intakeMotorVoltage;
        public double indexerMotorVoltage;
        public double intakeAmps;
        public double indexerAmps;
        public double intakeRPM;
        public double indexerRPM;
        public boolean sensorStatus;
    }

    public default void updateInputs(IntakeInputs inputs) {}

    public default void setIntakeMotorPercentage(double percent) {}

    public default void setIndexerMotorPercentage(double percent) {}
}
