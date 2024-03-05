package frc.robot;

/** Singleton tracker for operator state. */
public class OperatorState {

    private OperatorState() {}

    /** Operating state. */
    public static enum State {
        kAmp("Amp"), kShootPost("Shoot from Post"), kShootTrap("Shoot from Trap"), kShootWhileMove(
            "Shoot while moving"), kClimb("Climb");

        public final String displayName;

        State(String displayName) {
            this.displayName = displayName;
        }

        /** Get next state. */
        public State increment() {
            int new_ordinal = this.ordinal() + 1;
            if (new_ordinal >= State.values().length) {
                return this;
            }
            return State.values()[new_ordinal];
        }

        /** Get previous state. */
        public State decrement() {
            int new_ordinal = this.ordinal() - 1;
            if (new_ordinal < 0) {
                return this;
            }
            return State.values()[new_ordinal];
        }
    }

    private static State currentState = State.kAmp;
    private static boolean manualMode = false;

    /** Get whether or not operator should be able to control wrist and elevator manually. */
    public static boolean manualModeEnabled() {
        return manualMode;
    }

    /** Toggle manual mode for elevator/wrist. */
    public static void toggleManualMode() {
        manualMode = !manualMode;
    }

    /** Get currently tracked state. */
    public static State getCurrentState() {
        return currentState;
    }

    /** Advance state forward by one. */
    public static void increment() {
        currentState = currentState.increment();
    }

    /** Advance state backward by one. */
    public static void decrement() {
        currentState = currentState.decrement();
    }

    /**
     * Only use certain tags in certain modes. Helps for incorrect field layouts during practice.
     */
    public static boolean tagFilter(int id) {
        switch (getCurrentState()) {
            case kAmp:
                switch (id) {
                    case 5:
                    case 6:
                        return true;
                    default:
                        break;
                }
                break;
            case kShootWhileMove:
                switch (id) {
                    case 4:
                    case 7:
                        return true;
                    default:
                        break;
                }
                break;
            default:
                return true;
        }
        return false;
    }

}
