package frc.lib.util;

import frc.robot.Constants;

public class Deadzone {

    public static double applyDeadzone(double input) {
        return (Math.abs(input) < Constants.stickDeadband) ? 0
            : (input - Constants.stickDeadband) / (1.0 - Constants.stickDeadband);
    }

}
