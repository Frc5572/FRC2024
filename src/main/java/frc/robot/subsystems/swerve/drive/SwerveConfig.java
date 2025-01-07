package frc.robot.subsystems.swerve.drive;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.swerve.mod.ModuleConfig;
import frc.robot.subsystems.swerve.mod.ModuleConstants;
import lombok.Builder;

@Builder
public class SwerveConfig {

    public final ModuleConfig frontLeft;
    public final ModuleConfig frontRight;
    public final ModuleConfig backLeft;
    public final ModuleConfig backRight;

    public final ModuleConstants moduleConstants;

    public final Mass robotMass;
    public final MomentOfInertia robotMomentOfInertia;

    public ModuleConfig[] modules() {
        return new ModuleConfig[] { frontLeft, frontRight, backLeft, backRight };
    }

    public DriveTrainSimulationConfig getMapleConfig() {
        return DriveTrainSimulationConfig.Default().withRobotMass(robotMass)
            .withGyro(COTS.ofNav2X())
            .withSwerveModule(new SwerveModuleSimulationConfig(
                moduleConstants.driveMotor,
                moduleConstants.angleMotor,
                moduleConstants.driveReduction,
                moduleConstants.angleReduction,
                moduleConstants.driveFrictionVoltage,
                moduleConstants.angleFrictionVoltage,
                moduleConstants.wheelRadius,
                moduleConstants.angleMomentOfInertia,
                moduleConstants.wheelCoeffFriction));
    }

}
