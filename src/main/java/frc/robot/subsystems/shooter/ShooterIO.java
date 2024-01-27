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
        public double topshooterVelocityRotPerSecond;
        public double bottomshooterVelocityRotPerSecond;
    }

    public default void setTopMotor(double power) {}

    public default void setBottomMotor(double power) {}

    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}
}


