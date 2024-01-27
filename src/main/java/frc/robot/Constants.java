package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

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
         * Class for elevator and wrist motor constants
         */
        public static final class ElevatorWrist {
            public static final int TALON_ID = -1;
            public static final int NEO_ID = -1;
        }
    }

    /**
     * Swerve Constants
     */
    public static final class Swerve {
        public static final edu.wpi.first.wpilibj.SPI.Port navXID =
            edu.wpi.first.wpilibj.SPI.Port.kMXP;
        public static final boolean invertGyro = false;
        public static final boolean isFieldRelative = true;
        public static final boolean isOpenLoop = false;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.5);
        public static final double wheelBase = Units.inchesToMeters(29.0);
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
        public static final double angleGearRatio = 12.8 / 1.0;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert =
            InvertedValue.CounterClockwise_Positive;
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
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /**
         * Front Left Module - Module 0.
         */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 6;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CAN_CODER_ID = 0;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(138.604);
        }

        /**
         * Front Right Module - Module 1.
         */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final int CAN_CODER_ID = 0;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(280.107);
        }

        /**
         * Back Left Module - Module 2.
         */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CAN_CODER_ID = 0;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(121.553);
        }

        /**
         * Back Right Module - Module 3.
         */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 5;
            public static final int CAN_CODER_ID = 0;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(248.027);
        }

        public static final HolonomicPathFollowerConfig pathFollowerConfig =
            new HolonomicPathFollowerConfig(new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                // Drive base radius (distance from center to furthest module)
                maxSpeed, MOD0_MODOFFSET.getNorm(), new ReplanningConfig());
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
        public static final int ELEVATOR_ENC_CHANNEL_A = -1;
        public static final int ELEVATOR_ENC_CHANNEL_B = -1;
        public static final int TOP_LIMIT_SWITCH_PORT = -1;
        public static final int BOTTOM_LIMIT_SWITCH_PORT = -1;

        public static final double ELEVATOR_KP = 0;
        public static final double ELEVATOR_KI = 0;
        public static final double ELEVATOR_KD = 0;
        public static final double ELEVATOR_MAX_VELOCITY = 0;
        public static final double ELEVATOR_MAX_ACCELERATION = 0;
        public static final double ELEVATOR_KS = 0;
        public static final double ELEVATOR_KG = 0;
        public static final double ELEVATOR_KV = 0;


        public static final double WRIST_KP = 0;
        public static final double WRIST_KI = 0;
        public static final double WRIST_KD = 0;
        public static final double WRIST_MAX_VELOCITY = 0;
        public static final double WRIST_MAX_ACCELERATION = 0;
        public static final double WRIST_KS = 0;
        public static final double WRIST_KG = 0;
        public static final double WRIST_KV = 0;
    }
    /**
     * Pneumatics CAN id constants.
     */

    public static final class Pneumatics {
    }



}
