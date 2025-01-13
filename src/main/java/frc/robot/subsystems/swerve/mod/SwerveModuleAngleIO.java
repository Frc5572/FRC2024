package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

/** Interface for Angle Motors */
public interface SwerveModuleAngleIO {

    /** Inputs for Angle Motors */
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

    /** Update the angle motor inputs. */
    public void updateAngleInputs(SwerveModuleAngleInputs inputs);

    /** Change the PID constants for the angle motor. */
    public void setAnglePID(double p, double i, double d);

    /** Run the angle motor with a given output. */
    public void runAngleOpenLoop(double output);

    /** Run the angle motor using PID to a given rotation. */
    public void runAnglePosition(Rotation2d rotation);

}
