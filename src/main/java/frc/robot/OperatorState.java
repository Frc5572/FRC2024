package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.FieldConstants;

/** Singleton tracker for operator state. */
public class OperatorState {

    private OperatorState() {}

    /** Shooting state. */
    public static enum ShootingState {
        kAmp("Amp"), kShootPost("Shoot from Post"), kShootTrap("Shoot from Trap"), kShootWhileMove(
            "Shoot while moving"), kClimb("Climb");

        public final String displayName;

        ShootingState(String displayName) {
            this.displayName = displayName;
        }

        /** Get next state. */
        public ShootingState increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= ShootingState.values().length) {
                return this;
            }
            return ShootingState.values()[new_ordinal];
        }

        /** Get previous state. */
        public ShootingState decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal < 0) {
                return this;
            }
            return ShootingState.values()[new_ordinal];
        }
    }

    /** Climber state. */
    public static enum ClimberState {
        kStageLeftBlue("Stage Left Blue", 15), kStageRightBlue("Stage Right Blue",
            16), kCenterStageBlue("Center Stage Blue", 14);

        public final String displayName;
        public final int aprilTagId;

        ClimberState(String displayName, int aprilTagId) {
            this.displayName = displayName;
            this.aprilTagId = aprilTagId;
        }

        /** Get next state. */
        public ClimberState increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= ClimberState.values().length) {
                return this;
            }
            return ClimberState.values()[new_ordinal];
        }

        /** Get previous state. */
        public ClimberState decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal < 0) {
                return this;
            }
            return ClimberState.values()[new_ordinal];
        }

        public Pose2d getPose() {
            if (aprilTagId == 11) {
                return new Pose2d(
                    FieldConstants.aprilTags.getTagPose(7).get().toPose2d().getTranslation()
                        .plus(new Translation2d(20, new Rotation2d(Units.degreesToRadians(120)))),
                    Rotation2d.fromDegrees(0));
            } else {
                return null;
            }
        }
    }

    private static ShootingState ShootingCurrentState = ShootingState.kAmp;
    private static ClimberState ClimberCurrentState = ClimberState.kCenterStageBlue;
    private static boolean manualMode = false;

    /** Get whether or not operator should be able to control wrist and elevator manually. */
    public static boolean manualModeEnabled() {
        return manualMode;
    }

    /** Toggle manual mode for elevator/wrist. */
    public static void toggleManualMode() {
        manualMode = !manualMode;
    }

    /** Get currently tracked shooting state. */
    public static ShootingState getCurrentShootingState() {
        return ShootingCurrentState;
    }

    /** Advance shooting state forward by one. */
    public static void shootingIncrement() {
        ShootingCurrentState = ShootingCurrentState.increment();
    }

    /** Advance shooting state backward by one. */
    public static void shootingDecrement() {
        ShootingCurrentState = ShootingCurrentState.decrement();
    }

    /** Get currently tracked climber state. */
    public static ClimberState getCurrentClimberState() {
        return ClimberCurrentState;
    }

    /** Advance climber state forward by one. */
    public static void climberIncrement() {
        ClimberCurrentState = ClimberCurrentState.increment();
    }

    /** Advance climber state backward by one. */
    public static void climberDecrement() {
        ClimberCurrentState = ClimberCurrentState.decrement();
    }

}
