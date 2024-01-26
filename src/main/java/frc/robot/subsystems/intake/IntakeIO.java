package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeInputs {
        public double intakePercent;
        public double indexerPercent;
        public boolean sensorStatus;
    }

    public default void updateInputs(IntakeInputs inputs) {}

    public default void setIntakeMotorPercentage(double percent) {}

    public default void setIndexerMotorPercentage(double percent) {}
}
