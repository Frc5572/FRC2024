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
        public double topShooterVelocityRotPerMin;
        public double bottomShooterVelocityRotPerMin;
        public double topShooterSupplyVoltage;
        public double bottomShooterSupplyVoltage;
        public double topShooterAmps;
        public double bottomShooterAmps;
        public double topShooterPosition;
        public double bottomShooterPosition;
        public double topShooterPower;
        public double bottomShooterPower;
        public double topShooterTemp;
        public double bottomShooterTemp;
    }

    public default void setTopMotor(double power) {}

    public default void setBottomMotor(double power) {}

    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}
}


