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
        public static final class ElevatorWrist {
            public static final int TALON_ID = -1;
            public static final int NEO_ID = -1;
        }
    }

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
