package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleDriveIO {

    @AutoLog
    public static class SwerveModuleDriveInputs {
        public boolean motorConnected;
        public double positionRads;
        public double velocityRadsPerSec;
        public double appliedVolts;
        public double supplyCurrentAmps;
        public double[] odometryDrivePositionsMeters = new double[] {};
    }

    public void updateDriveInputs(SwerveModuleDriveInputs inputs);

    public void setDrivePID(double p, double i, double d);

    public void runDriveOpenLoop(double output);

    public void runDriveVelocity(double velocityRadPerSec, double feedforward);

    public void setBrakeMode(boolean enabled);

}
