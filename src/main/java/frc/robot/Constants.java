package frc.robot;

import static edu.wpi.first.units.Units.*;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.swerve.mod.ModuleConfig;

/**
 * Constants file.
 */
public final class Constants {

    public static final boolean tuningMode = false;

    /**
     * Stick Deadband
     */
    public static final double stickDeadband = 0.1;

    /**
     * Driver ID
     */
    public static final int driverId = 0;

    /**
     * Operator ID
     */
    public static final int operatorId = 1;

    /**
     * Swerve Constants
     */
    public static final class Swerve {

        public static final double odometryFrequency = 250;

        public static class ModuleConstants {

            public static final DCMotor driveMotor = DCMotor.getKrakenX60(1);
            public static final DCMotor angleMotor = DCMotor.getFalcon500(1);
            public static final Voltage driveFrictionVoltage = Volts.of(0);
            public static final Voltage angleFrictionVoltage = Volts.of(0);
            public static final double wheelCoeffFriction = 1.0;
            public static final MomentOfInertia angleMomentOfInertia =
                KilogramSquareMeters.of(0.02);
            public static final Distance wheelRadius = Inches.of(1.9);
            public static final Current slipCurrent = Amps.of(40.0);
            public static final Current supplyCurrentLimit = Amps.of(35.0);
            public static final Current supplyCurrentLowerLimit = Amps.of(60.0);
            public static final Time supplyCurrentLowerTimeThreshold = Seconds.of(0.1);

            public static final AngularVelocity maxSteerRate = RotationsPerSecond.of(4.0);
            public static final LinearAcceleration maxDriveRate = MetersPerSecondPerSecond.of(50.0);


            public static final double ffkS = 0.32;
            public static final double ffkV = 1.51;
            public static final double ffkT = 1.0 / driveMotor.KtNMPerAmp;
            public static final double ffkA = 0.27;
            public static final double drivekP = 0.12;
            public static final double drivekD = 0.0;
            public static final double anglekP = 100.0;
            public static final double anglekD = 0.0;
            public static final double driveReduction = Mk4iReductions.L3.reduction;
            public static final double angleReduction = Mk4iReductions.TURN.reduction;
        }

        public static final int gyroId = 0;

        public static final ModuleConfig frontLeft = ModuleConfig.builder()
            .moduleNumber(0)
            .driveId(6)
            .angleId(51)
            .absoluteEncoderId(4)
            .absoluteEncoderOffset(Rotation2d.fromRotations(-0.496826))
            .angleMotorInverted(false)
            .build();
        public static final ModuleConfig frontRight = ModuleConfig.builder()
            .moduleNumber(1)
            .driveId(2)
            .angleId(40)
            .absoluteEncoderId(2)
            .absoluteEncoderOffset(Rotation2d.fromRotations(0.405518 + 0.5))
            .angleMotorInverted(false)
            .build();
        public static final ModuleConfig backLeft = ModuleConfig.builder()
            .moduleNumber(2)
            .driveId(3)
            .angleId(9)
            .absoluteEncoderId(1)
            .absoluteEncoderOffset(Rotation2d.fromRotations(0.348145))
            .angleMotorInverted(false)
            .build();
        public static final ModuleConfig backRight = ModuleConfig.builder()
            .moduleNumber(3)
            .driveId(10)
            .angleId(8)
            .absoluteEncoderId(10)
            .absoluteEncoderOffset(Rotation2d.fromRotations(0.317627 + 0.5))
            .angleMotorInverted(false)
            .build();

        public static final Mass robotMass = Pounds.of(150.0);
        public static final MomentOfInertia robotMomentOfInertia = KilogramSquareMeters.of(6.8);

        public static final Distance trackWidthX = Inches.of(23.75);
        public static final Distance trackWidthY = Inches.of(17.75);

        public static final LinearVelocity scrubLimit = MetersPerSecond.of(0.1);
        public static final LinearVelocity maxLinearSpeed = MetersPerSecond.of(10.0);

        public static ModuleConfig[] modules() {
            return new ModuleConfig[] { frontLeft, frontRight, backLeft, backRight };
        }

        public static Translation2d[] getModuleTranslations() {
            return new Translation2d[] {
                new Translation2d(trackWidthX.in(Units.Meters) / 2,
                    trackWidthY.in(Units.Meters) / 2),
                new Translation2d(trackWidthX.in(Units.Meters) / 2,
                    -trackWidthY.in(Units.Meters) / 2),
                new Translation2d(-trackWidthX.in(Units.Meters) / 2,
                    trackWidthY.in(Units.Meters) / 2),
                new Translation2d(-trackWidthX.in(Units.Meters) / 2,
                    -trackWidthY.in(Units.Meters) / 2)
            };
        }

        public static Distance getDriveBaseRadius() {
            return Units.Meters
                .of(Math.hypot(trackWidthX.in(Units.Meters) / 2.0,
                    trackWidthY.in(Units.Meters) / 2.0));
        }

        public static DriveTrainSimulationConfig getMapleConfig() {
            return DriveTrainSimulationConfig.Default()
                .withRobotMass(robotMass)
                .withGyro(COTS.ofNav2X())
                .withCustomModuleTranslations(getModuleTranslations())
                .withSwerveModule(new SwerveModuleSimulationConfig(
                    ModuleConstants.driveMotor,
                    ModuleConstants.angleMotor,
                    ModuleConstants.driveReduction,
                    ModuleConstants.angleReduction,
                    ModuleConstants.driveFrictionVoltage,
                    ModuleConstants.angleFrictionVoltage,
                    ModuleConstants.wheelRadius,
                    ModuleConstants.angleMomentOfInertia,
                    ModuleConstants.wheelCoeffFriction));
        }

        private enum Mk4iReductions {
            L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)), L3(
                (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)), TURN((150.0 / 7.0));

            final double reduction;

            Mk4iReductions(double reduction) {
                this.reduction = reduction;
            }
        }
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
