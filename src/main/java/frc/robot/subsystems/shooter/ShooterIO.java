package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Class for ShooterIO
 */
public interface ShooterIO {

    /**
     * Inputs of ShooterIO
     */
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterVelocityRotPerMin;
        public double shooterSupplyVoltage;
        public double shooterAmps;

    }

    public default void setMotor(double power) {}

    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}
}


