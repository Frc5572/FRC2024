package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

/** Command to flash LEDs on and off once per second. */
public class FlashingLEDColor extends Command {
    private LEDs leds;
    private int ledLength;
    private int flashingDelay = 0;
    private Color color;
    private Color altColor;

    /**
     * Command to flash the LED strip between 2 colors
     *
     * @param leds LED Subsystem
     * @param color The first color
     * @param altColor The second color
     */
    public FlashingLEDColor(LEDs leds, Color color, Color altColor) {
        this.leds = leds;
        this.color = color;
        this.altColor = altColor;
        ledLength = leds.getLength();
        addRequirements(leds);
    }

    /**
     * Command to flash the LED strip between a color and black
     *
     * @param leds LED Subsystem
     * @param color The color
     */
    public FlashingLEDColor(LEDs leds, Color color) {
        this(leds, color, Color.kBlack);
    }

    @Override
    public void execute() {
        if (flashingDelay < 10) {
            for (var i = 0; i < ledLength; i++) {
                leds.setColor(i, color);
            }
        } else {
            for (var i = 0; i < ledLength; i++) {
                leds.setColor(i, altColor);
            }
        }
        leds.setData();
        flashingDelay++;
        flashingDelay %= 20;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
