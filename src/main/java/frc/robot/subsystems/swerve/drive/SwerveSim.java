package frc.robot.subsystems.swerve.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;

public class SwerveSim implements SwerveIO {

    private final GyroSimulation simulation;

    public SwerveSim(GyroSimulation simulation) {
        this.simulation = simulation;
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = simulation.getGyroReading();
        inputs.yawVelocityRadPerSec =
            simulation.getMeasuredAngularVelocity().in(Units.RadiansPerSecond);
        inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
    }

}
