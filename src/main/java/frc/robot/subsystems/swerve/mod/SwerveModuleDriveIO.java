package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.AutoLog;

/** Interface for Drive Motors */
public interface SwerveModuleDriveIO {

    /** Inputs for Drive Motors */
    @AutoLog
    public static class SwerveModuleDriveInputs {
        public boolean motorConnected;
        public double positionRads;
        public double velocityRadsPerSec;
        public double appliedVolts;
        public double supplyCurrentAmps;
        public double[] odometryDrivePositionsMeters = new double[] {};
        public double[] odometryTimestamps = new double[] {};
    }

    /** Update the drive motor inputs. */
    public void updateDriveInputs(SwerveModuleDriveInputs inputs);

    /** Change the PID constants for the drive motor. */
    public void setDrivePID(double p, double i, double d);

    /** Run the drive motor with a given output. */
    public void runDriveOpenLoop(double output);

    /** Run the angle motor using PID to a given velocity. */
    public void runDriveVelocity(double velocityRadPerSec, double feedforward);

    /** Set the brake mode for the drive motor. */
    public void setBrakeMode(boolean enabled);

}
