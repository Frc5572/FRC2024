package frc.robot;

/**
 * Constants file.
 */
public final class Constants {
    /**
     * Stick Deadband
     */
    public static final double stickDeadband = 0.1;
    /**
     * Driver ID
     */
    public static final int driverID = 0;
    /**
     * Operator ID
     */
    public static final int operatorID = 1;

    /**
     * Motor CAN id's.
     */
    public static final class Motors {
        public static final class Shooter {
            public static final int shooterTopId = 0;
            public static final int shooterBottomId = 0;
        }
    }

    /**
     * Pneumatics CAN id constants.
     */
    public static final class Pneumatics {
    }

    public static final class ShooterConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }


}
