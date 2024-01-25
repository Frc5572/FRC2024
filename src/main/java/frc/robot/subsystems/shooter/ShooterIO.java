package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public static class ShooterIOInputs{
        public double shooterVelocityRotPerSecond;
        
        
    }
    public default void setTopMotor(double power) {

    }
    public default void setBottomMotor(double power) {

    }
    public default void updateInputs(ShooterIOInputs inputs){

    }
}
