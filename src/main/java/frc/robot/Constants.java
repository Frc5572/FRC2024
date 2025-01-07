package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.swerve.drive.SwerveConfig;
import frc.robot.subsystems.swerve.mod.ModuleConfig;
import frc.robot.subsystems.swerve.mod.ModuleConstants;

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

        public static final SwerveConfig config = SwerveConfig.builder()
            .moduleConstants(
                ModuleConstants.builder()
                    .ffkS(5.0)
                    .ffkV(0.0)
                    .ffkT(1.0 / DCMotor.getKrakenX60(1).KtNMPerAmp)
                    .drivekP(35.0)
                    .drivekD(0.0)
                    .anglekP(4000.0)
                    .anglekD(50.0)
                    .driveReduction(Mk4iReductions.L3.reduction)
                    .angleReduction(Mk4iReductions.TURN.reduction)
                    .driveMotor(DCMotor.getKrakenX60(1))
                    .angleMotor(DCMotor.getFalcon500(1))
                    .driveFrictionVoltage(Volts.of(0))
                    .angleFrictionVoltage(Volts.of(0))
                    .wheelCoeffFriction(1.0)
                    .angleMomentOfInertia(KilogramSquareMeters.of(0.2))
                    .wheelRadius(Inches.of(1.8))
                    .slipCurrent(Amps.of(40.0))
                    .build())
            .robotMass(Pounds.of(150.0))
            .robotMomentOfInertia(KilogramSquareMeters.of(6.8))
            .frontLeft(ModuleConfig.builder()
                .moduleNumber(0)
                .driveId(0)
                .angleId(0)
                .absoluteEncoderId(0)
                .absoluteEncoderOffset(Rotation2d.fromRotations(0.0))
                .angleMotorInverted(false)
                .build())
            .frontRight(ModuleConfig.builder()
                .moduleNumber(0)
                .driveId(0)
                .angleId(0)
                .absoluteEncoderId(0)
                .absoluteEncoderOffset(Rotation2d.fromRotations(0.0))
                .angleMotorInverted(false)
                .build())
            .backLeft(ModuleConfig.builder()
                .moduleNumber(0)
                .driveId(0)
                .angleId(0)
                .absoluteEncoderId(0)
                .absoluteEncoderOffset(Rotation2d.fromRotations(0.0))
                .angleMotorInverted(false)
                .build())
            .backRight(ModuleConfig.builder()
                .moduleNumber(0)
                .driveId(0)
                .angleId(0)
                .absoluteEncoderId(0)
                .absoluteEncoderOffset(Rotation2d.fromRotations(0.0))
                .angleMotorInverted(false)
                .build())
            .build();

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
