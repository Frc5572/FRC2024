package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

/**
 * Command to flash the LED strip between red and blue like police lights
 */
public class PoliceLEDs extends Command {
    private LEDs leds;
    private int policeDelay = 0;
    private int ledLength;
    private int ledLengthHalf;

    /**
     * Command to flash the LED strip between red and blue like police lights
     *
     * @param leds LED subsystem
     */
    public PoliceLEDs(LEDs leds) {
        this.leds = leds;
        ledLength = leds.getLength();
        ledLengthHalf = ledLength / 2;
        addRequirements(leds);
    }

    @Override
    public void execute() {
        if (policeDelay < 10) {
            for (var i = 0; i < ledLengthHalf; i++) {
                leds.setColor(i, Color.kRed);
            }
            for (var i = ledLengthHalf; i < ledLength; i++) {
                leds.setColor(i, Color.kBlack);
            }
        } else {
            for (var i = 0; i < ledLengthHalf; i++) {
                leds.setColor(i, Color.kBlack);
            }
            for (var i = ledLengthHalf; i < ledLength; i++) {
                leds.setColor(i, Color.kBlue);
            }
        }
        leds.setData();
        policeDelay += 1;
        policeDelay %= 21;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

