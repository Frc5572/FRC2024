package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.FieldConstants;

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
     * Motor CAN id's.
     */
    public static final class Motors {
        /**
         * Shooter Id's
         */
        public static final class Shooter {
            public static final int SHOOTER_TOP_ID = 13;
            public static final int SHOOTER_BOTTOM_ID = 15;
        }

        /**
         * Class for elevator and wrist motor constants
         */
        public static final class ElevatorWrist {
            public static final int ELEVATOR_NEO_ID = 57;
            public static final int WRIST_NEO_ID = 52;
        }

        /**
         * Intake and indexer motor constants
         */
        public static final class Intake {
            public static final int INTAKE_MOTOR_ID = 14;
            public static final int INDEXER_MOTOR_ID = 32;
        }

        /**
         * Climber motor constants
         */
        public static final class Climber {
            public static final int LEFT_MOTOR_ID = 60;
            public static final int RIGHT_MOTOR_ID = 48;
        }
    }

    /**
     * Camera offset constants.
     */
    public static class CameraConstants {

        /**
         * Constants for Front Left Camera
         */
        public static class FrontLeftFacingCamera {
            public static final double ROLL = 0.0;
            public static final double PITCH = Math.toRadians(40);
            public static final double YAW = Math.toRadians(10);
            public static final Transform3d KCAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(Units.inchesToMeters(-3.96),
                    Units.inchesToMeters(10.275), Units.inchesToMeters(18.164)),
                    new Rotation3d(ROLL, PITCH, YAW)).inverse();

            public static final String CAMERA_NAME = "front-left";
            public static final double LARGEST_DISTANCE = 0.1;
        }

        /**
         * Constants for Front Right Camera
         */
        public static class FrontRightFacingCamera {
            public static final double ROLL = 0.0;
            public static final double PITCH = Math.toRadians(5);
            public static final double YAW = Math.toRadians(-10);
            public static final Transform3d KCAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(Units.inchesToMeters(3.96),
                    Units.inchesToMeters(11.013), Units.inchesToMeters(18.074)),
                    new Rotation3d(ROLL, PITCH, YAW)).inverse();

            public static final String CAMERA_NAME = "front-right";
            public static final double LARGEST_DISTANCE = 0.1;
        }

        /**
         * Constants for Back Left Camera
         */
        public static class BackLeftFacingCamera {
            public static final double ROLL = 0.0;
            public static final double PITCH = Math.toRadians(0);
            public static final double YAW = Math.toRadians(180);
            public static final Transform3d KCAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(Units.inchesToMeters(-12.83),
                    Units.inchesToMeters(-8.175), Units.inchesToMeters(18.5)),
                    new Rotation3d(ROLL, PITCH, YAW)).inverse();

            public static final String CAMERA_NAME = "back-left";
            public static final double LARGEST_DISTANCE = 0.1;
        }

        /**
         * Constants for Back Right Camera
         */
        public static class BackRightFacingCamera {
            public static final double ROLL = 0.0;
            public static final double PITCH = Math.toRadians(0);
            public static final double YAW = Math.toRadians(180);
            public static final Transform3d KCAMERA_TO_ROBOT =
                new Transform3d(new Translation3d(Units.inchesToMeters(12.831),
                    Units.inchesToMeters(-8.56), Units.inchesToMeters(17.85)),
                    new Rotation3d(ROLL, PITCH, YAW)).inverse();

            public static final String CAMERA_NAME = "back-right";
            public static final double LARGEST_DISTANCE = 0.1;
        }

    }

    /**
     * Swerve Constants
     */
    public static final class Swerve {
        public static final edu.wpi.first.wpilibj.SPI.Port navXID =
            edu.wpi.first.wpilibj.SPI.Port.kMXP;
        public static final boolean invertGyro = true;
        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = false;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.75);
        public static final double wheelBase = Units.inchesToMeters(17.75);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final Translation2d MOD0_MODOFFSET =
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0);

        /*
         * Swerve Kinematics No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = (8.14 / 1.0); // MK4i L1
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // (150 / 7) : 1

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue driveMotorInvert =
            InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 100.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 1.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */

        /**
         * Front Left Module - Module 3
         */
        public static final class Mod0 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 51;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(3.955078125);
        }

        /**
         * Front Right Module - Module 1
         */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 40;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(145.01953125);
        }

        /**
         * Back Left Module - Module 2
         */
        public static final class Mod2 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-55.37109375);
        }

        /**
         * Back Right Module - Module 3
         */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(115.400390625);
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig =
            new HolonomicPathFollowerConfig(new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                // Drive base radius (distance from center to furthest module)
                maxSpeed, MOD0_MODOFFSET.getNorm(), new ReplanningConfig());
    }

    /**
     * Climber constants
     */
    public static final class ClimberConstants {
        public static final double CLIMBER_KP = 0.1;
        public static final double CLIMBER_KI = 0.1;
        public static final double CLIMBER_KD = 0.1;
        public static final double CLIMBER_MAX_VELOCITY = 0;
        public static final double CLIMBER_MAX_ACCELERATION = 0;
        public static final double CLIMBER_KS = 0.1;
        public static final double CLIMBER_KG = 0.1;
        public static final double CLIMBER_KV = 0.1;

        public static final double CLIMBING_DISTANCE = Units.inchesToMeters(15);
        public static final double MAX_CLIMBING_DISTANCE = Units.inchesToMeters(21);

        // 2pi * radius
        public static final double LINEAR_DISTANCE = Units.inchesToMeters(2 * Math.PI * 1);
    }

    /**
     * Auto constants
     */
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);
    }

    /**
     * Class for elevator and wrist constants
     */
    public static final class ElevatorWristConstants {

        /**
         * Sensor Constants
         */
        public static final class Sensors {

            public static final int ELEVATOR_ENC_CHANNEL_A = -1;
            public static final int ELEVATOR_ENC_CHANNEL_B = -1;
            public static final int TOP_LIMIT_SWITCH_PORT = 1;
            public static final int BOTTOM_LIMIT_SWITCH_PORT = 2;
        }

        /**
         * PID constants
         */
        public static final class PID {

            public static final double ELEVATOR_KP = 0;
            public static final double ELEVATOR_KI = 0;
            public static final double ELEVATOR_KD = 0;
            public static final double ELEVATOR_MAX_VELOCITY = 0;
            public static final double ELEVATOR_MAX_ACCELERATION = 0;
            public static final double ELEVATOR_KS = 0;
            public static final double ELEVATOR_KG = 0;
            public static final double ELEVATOR_KV = 0;

            public static final double WRIST_KP = 1.8 / 180;
            public static final double WRIST_KI = 0;
            public static final double WRIST_KD = 0;
            public static final double WRIST_MAX_VELOCITY = 0;
            public static final double WRIST_MAX_ACCELERATION = 0;
            public static final double WRIST_KS = 0;
            public static final double WRIST_KG = 0;
            public static final double WRIST_KV = 0;
        }

        /**
         * Set points constants for elevator and wrist
         */
        public static final class SetPoints {

            public static final double HOME_HEIGHT = 0;
            public static final Rotation2d HOME_ANGLE = Rotation2d.fromDegrees(40);
            public static final double AMP_HEIGHT = Units.inchesToMeters(34);
            public static final Rotation2d AMP_ANGLE = Rotation2d.fromDegrees(-10);
            public static final double TRAP_HEIGHT = Units.inchesToMeters(40);
            public static final Rotation2d TRAP_ANGLE = Rotation2d.fromDegrees(30);
            public static final double MAX_EXTENSION = Units.inchesToMeters(48);
            public static final double CLIMBING_HEIGHT = Units.inchesToMeters(15);
            public static final Rotation2d CLIMBING_ANGLE = Rotation2d.fromDegrees(0);
            public static final Rotation2d MAX_ANGLE_UP_HOME = Rotation2d.fromDegrees(75);
            public static final Rotation2d MAX_ANGLE_DOWN_HOME = Rotation2d.fromDegrees(0);
            public static final Rotation2d MAX_ANGLE_UP_EXTENDED = Rotation2d.fromDegrees(75);
            public static final Rotation2d MAX_ANGLE_DOWN_EXTENDED = Rotation2d.fromDegrees(-15);

            public static final double LINEAR_DISTANCE = Units.inchesToMeters(2 * Math.PI * 659);

        }


    }
    /**
     * Pneumatics CAN id constants.
     */

    public static final class Pneumatics {
    }



    /**
     * Constants of Shooters
     */
    public static final class ShooterConstants {
        public static final double KP = 0;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0;
        public static final double KV = 0;
        public static final double HEIGHT_FROM_LOWEST_POS = Units.inchesToMeters(32.0);
        public static final double HEIGHT_FROM_SPEAKER =
            FieldConstants.centerSpeaker - HEIGHT_FROM_LOWEST_POS;
    }

    /**
     * Constants for intake
     */
    public static final class IntakeConstants {
        public static final double INTAKE_MOTOR_FORWARD = 0;
        public static final double INTAKE_MOTOR_BACKWARD = -0;
        public static final double INTAKE_MOTOR_STOP = 0;
        public static final double INDEX_MOTOR_FORWARD = .2;
        public static final double INDEX_MOTOR_BACKWARD = -0;
        public static final double INDEX_MOTOR_STOP = 0;
        public static final boolean INTAKE_MOTOR_INVERTED = true;

    }
}
