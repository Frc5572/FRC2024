package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleAngleIO {

    @AutoLog
    public static class SwerveModuleAngleInputs {
        public boolean motorConnected = true;
        public boolean encoderConnected = true;
        public Rotation2d absolutePosition = new Rotation2d();
        public Rotation2d position = new Rotation2d();
        public double velocityRadsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public void updateAngleInputs(SwerveModuleAngleInputs inputs);

    public void setAnglePID(double p, double i, double d);

    public void runAngleOpenLoop(double output);

    public void runAnglePosition(Rotation2d rotation);

}
