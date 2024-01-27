package frc.robot;

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

        /**
         * Intake and indexer motor constants
         */
        public static final class Intake {
            public static final int INTAKE_MOTOR_ID = -1;
            public static final int INDEXER_MOTOR_ID = -1;
        }

        /**
         * Climber motor constants
         */
        public static final class Climber {
            public static final int CLIMBER_MOTOR_ID = -1;
            public static final int CLIMBER_MOTOR_ID2 = -1;
        }
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
     * Climber constants
     */
    public static final class ClimberConstants {
        public static final double CLIMBER_KP = 0;
        public static final double CLIMBER_KI = 0;
        public static final double CLIMBER_KD = 0;
        public static final double CLIMBER_MAX_VELOCITY = 0;
        public static final double CLIMBER_MAX_ACCELERATION = 0;
        public static final double CLIMBER_KS = 0;
        public static final double CLIMBER_KG = 0;
        public static final double CLIMBER_KV = 0;
    }

    /**
     * Pneumatics CAN id constants.
     */
    public static final class Pneumatics {
    }
}
