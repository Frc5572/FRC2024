package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Command to Pulse the Left and Right Rumble on a controller
 */
public class RumblePulse extends Command {
    private final double frequency;
    private GenericHID controller;
    private double rumbleAmount = 1;
    private boolean swap = false;

    /**
     * Rumble the controller left and right every X tenths of a second
     *
     * @param controller The controller to rumble
     * @param frequency The frequency to swap rumble side in tenths of a second
     */
    public RumblePulse(CommandXboxController controller, double frequency) {
        this.frequency = frequency;
        this.controller = controller.getHID();
    }

    @Override
    public void execute() {
        if (swap) {
            this.controller.setRumble(RumbleType.kRightRumble, rumbleAmount);
            this.controller.setRumble(RumbleType.kLeftRumble, 0);
        } else {
            this.controller.setRumble(RumbleType.kLeftRumble, rumbleAmount);
            this.controller.setRumble(RumbleType.kRightRumble, 0);
        }
        if (Math.round(Timer.getFPGATimestamp() * 10) % frequency == 0) {
            swap = !swap;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.controller.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
