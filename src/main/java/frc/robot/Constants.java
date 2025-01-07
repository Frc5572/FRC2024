package frc.robot;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.swerve.drive.SwerveConfig;

/**
 * Constants file.
 */
public final class Constants {
    /**
     * Stick Deadband
     */
    public static final double STICK_DEADBAND = 0.1;

    /**
     * Driver ID
     */
    public static final int DRIVER_ID = 0;

    /**
     * Operator ID
     */
    public static final int OPERATOR_ID = 1;

    /**
     * Swerve Constants
     */
    public static final class Swerve {
        public static final double odometryFrequency = 250;

        public static final SwerveConfig config = new SwerveConfig();

        public static final Mass robotMass = Units.Pounds.of(140);
        public static final MomentOfInertia robotMomentOfInertia =
            Units.KilogramSquareMeters.of(6.88);
        public static final double wheelCoeffFriction = 1.0;

        public static final double driveGearRatio = 1.0;
        public static final Voltage driveFrictionVoltage = Units.Volts.of(1.0);

        public static final double angleGearRatio = 1.0;
        public static final Voltage angleFrictionVoltage = Units.Volts.of(1.0);
        public static final MomentOfInertia angleMomentOfInertia =
            Units.KilogramSquareMeters.of(0.0);

        public static final Distance wheelRadius = Units.Inches.of(1.8);

        public static final Current slipCurrent = Units.Amp.of(0.0);

        public static final DriveTrainSimulationConfig mapleSimConfig =
            DriveTrainSimulationConfig.Default().withRobotMass(robotMass).withGyro(COTS.ofNav2X())
                .withSwerveModule(new SwerveModuleSimulationConfig(DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1), driveGearRatio, angleGearRatio, driveFrictionVoltage,
                    angleFrictionVoltage, wheelRadius, angleMomentOfInertia, wheelCoeffFriction));
    }

    /**
     * LED constants.
     */
    public static final class LEDConstants {
        public static final int PWM_PORT = 9;
        public static final int LED_COUNT = 60;
        public static final Color INTAKE_COLOR = Color.kGreen;
        public static final Color INDEXER_COLOR = Color.kPurple;
        public static final Color ALERT_COLOR = Color.kWhite;
    }

}
