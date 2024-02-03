package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.subsystems.LEDs;

/**
 * Command to move LEDs back and forth like a Cylon eye
 */
public class FireLEDs extends Command {
    private final LEDs leds;
    private int end;
    private int start;
    private int cooling = 100;
    private int sparking = 50;
    private int speedDelay = 15;
    private int[] heat;

    /**
     * Command to make a fire-esque display for a single column of LEDs
     *
     * @param leds The LED subsystem
     * @param cooling Indicates how fast a flame cools down. More cooling means shorter flames.
     * @param sparking Indicates the chance (out of 255) that a spark will ignite. A higher value
     *        makes the fire more active.
     * @param speedDelay Delay in ms. Allows you to slow down the fire activity ... a higher value
     *        makes the flame appear slower
     */
    public FireLEDs(LEDs leds, int cooling, int sparking, int speedDelay) {
        this.leds = leds;
        this.end = leds.getLength();
        this.start = 0;
        this.cooling = cooling;
        this.sparking = sparking;
        this.speedDelay = speedDelay;
        this.heat = new int[leds.getLength()];
        addRequirements(leds);
    }

    /**
     * Command to make a fire-esque display for a single column of LEDs
     *
     * @param leds The LED subsystem
     */
    public FireLEDs(LEDs leds) {
        this(leds, 100, 50, 15);
    }

    @Override
    public void initialize() {
        this.heat = new int[leds.getLength()];
    }

    @Override
    public void execute() {
        // Step 1. Cool down every cell a little
        for (int i = start; i < end; i++) {
            int cooldown = Conversions.random(((cooling * 10) / leds.getLength()) + 2, 0);
            if (cooldown > heat[i]) {
                heat[i] = 0;
            } else {
                heat[i] = heat[i] - cooldown;
            }
        }

        // Step 2. Heat from each cell drifts 'up' and diffuses a little
        for (int i = end - 1; i >= start + 2; i--) {
            heat[i] = (int) ((heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3);
        }
        // Step 3. Randomly ignite new 'sparks' near the bottom
        if (Conversions.random(255, 0) < this.sparking) {
            int y = Conversions.random(7, 0);
            heat[y] = heat[y] + Conversions.random(255, 160);
        }
        // Step 4. Convert heat to LED colors
        for (int j = start; j < end; j++) {
            setPixelHeatColor(j, heat[j]);
        }
        leds.setData();
        Timer.delay(speedDelay / 1000.0);
    }


    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    /**
     * Set the "Heat" of the individual LED
     *
     * @param index The index of the LED to change
     * @param temperature The temperature of the LED
     */
    private void setPixelHeatColor(int index, int temperature) {
        // Scale 'heat' down from 0-255 to 0-191
        int t192 = (int) Math.round((temperature / 255.0) * 191);

        // calculate ramp up from
        int heatramp = (int) (t192 & 0x3F); // 0..63
        heatramp <<= 2; // scale up to 0..252

        // figure out which third of the spectrum we're in:
        if (t192 > 0x80) { // hottest
            leds.setRGB(index, 255, 255, heatramp);
        } else if (t192 > 0x40) { // middle
            leds.setRGB(index, 255, heatramp, 0);
        } else { // coolest
            leds.setRGB(index, heatramp, 0, 0);
        }
    }
}
