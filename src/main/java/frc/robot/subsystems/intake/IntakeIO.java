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
        public boolean intakeBeamBrake;
        public double intakeRPM;
    }

    public default void updateInputs(IntakeInputs inputs) {}

    public default void setIntakeMotorPercentage(double percent) {}
}
