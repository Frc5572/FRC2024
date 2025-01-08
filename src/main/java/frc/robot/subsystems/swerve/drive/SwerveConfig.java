package frc.robot.subsystems.swerve.drive;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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

    public final Distance trackWidthX;
    public final Distance trackWidthY;

    public final LinearVelocity scrubLimit;
    public final LinearVelocity maxLinearSpeed;

    public ModuleConfig[] modules() {
        return new ModuleConfig[] { frontLeft, frontRight, backLeft, backRight };
    }

    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(trackWidthX.in(Units.Meters) / 2, trackWidthY.in(Units.Meters) / 2),
            new Translation2d(trackWidthX.in(Units.Meters) / 2, -trackWidthY.in(Units.Meters) / 2),
            new Translation2d(-trackWidthX.in(Units.Meters) / 2, trackWidthY.in(Units.Meters) / 2),
            new Translation2d(-trackWidthX.in(Units.Meters) / 2, -trackWidthY.in(Units.Meters) / 2)
        };
    }

    public Distance getDriveBaseRadius() {
        return Units.Meters
            .of(Math.hypot(trackWidthX.in(Units.Meters) / 2.0, trackWidthY.in(Units.Meters) / 2.0));
    }

    public DriveTrainSimulationConfig getMapleConfig() {
        return DriveTrainSimulationConfig.Default()
            .withRobotMass(robotMass)
            .withGyro(COTS.ofNav2X())
            .withCustomModuleTranslations(getModuleTranslations())
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
