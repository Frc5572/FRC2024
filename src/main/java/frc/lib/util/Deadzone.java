package frc.lib.util;

import frc.robot.Constants;

/** Deadzone Utilities */
public class Deadzone {

    /** Make deadzone for an input axis, with proper scaling. */
    public static double applyDeadzone(double input) {
        return (Math.abs(input) < Constants.stickDeadband) ? 0
            : (input - Constants.stickDeadband) / (1.0 - Constants.stickDeadband);
    }

}
