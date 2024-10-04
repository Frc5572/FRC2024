package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/**
 * Intake IO Interface
 */
public interface IndexerIO {

    /**
     * Intake Inputs to Log
     */
    @AutoLog
    public static class IndexerInputs {
        public boolean indexerBeamBrake;
        public double indexerRPM;
    }

    public default void updateInputs(IndexerInputs inputs) {}

    public default void setIndexerMotorPercentage(double percent) {}
}
