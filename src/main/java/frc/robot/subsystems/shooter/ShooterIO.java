package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double shooterVelocityRotPerSecond;


    }

    public default void setTopMotor(double power) {

    }

    public default void setBottomMotor(double power) {

    }

    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {

    }
}


