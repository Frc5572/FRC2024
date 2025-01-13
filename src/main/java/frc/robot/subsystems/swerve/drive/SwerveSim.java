package frc.robot.subsystems.swerve.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;

/** Gyro implementation for Maple-Sim */
public class SwerveSim implements SwerveIO {

    private final GyroSimulation gyroSim;
    private final SwerveDriveSimulation simulation;

    /** Create new sim drivetrain */
    public SwerveSim(SwerveDriveSimulation simulation) {
        this.gyroSim = simulation.getGyroSimulation();
        this.simulation = simulation;
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSim.getGyroReading();
        inputs.yawVelocityRadPerSec =
            gyroSim.getMeasuredAngularVelocity().in(Units.RadiansPerSecond);
    }

    @Override
    public void overridePose(Pose2d pose) {
        simulation.setSimulationWorldPose(pose);
    }

}
