package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.subsystems.LEDs;

/**
 * Command to move LEDs back and forth like a Cylon eye
 *
 * <p>
 * Doesn't really work
 */
public class MeteorRain extends Command {
    private final LEDs leds;
    private int numLEDs;
    private boolean meteorRandomDecay = true;
    private int meteorTrailDecay = 64;
    private int meteorSize = 10;

    /**
     * Create a Meteor Rain effect like
     * https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/#LEDStripEffectMeteorRain
     *
     * @param leds The LED subsystem
     */
    public MeteorRain(LEDs leds) {
        this.leds = leds;
        this.numLEDs = leds.getLength();
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        leds.setColor(Color.kBlack);
    }

    @Override
    public void execute() {
        // leds.setColor(Color.kBlack);
        for (int i = 0; i < numLEDs + numLEDs; i++) {


            // fade brightness all LEDs one step
            for (int j = 0; j < numLEDs; j++) {
                if (!meteorRandomDecay || (Conversions.random(10) > 5)) {
                    fadeToBlack(j, meteorTrailDecay);
                }
            }

            // draw meteor
            for (int j = 0; j < meteorSize; j++) {
                if ((i - j < numLEDs) && (i - j >= 0)) {
                    leds.setColor(i - j, Color.kWhite);
                }
            }

            leds.setData();
            Timer.delay(30 / 1000.0);
        }
    }


    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    /**
     * Fade an individual LED to black
     *
     * @param index The index of the LED to change
     * @param fadeValue The amount to fade by.
     */
    private void fadeToBlack(int index, int fadeValue) {
        Color oldColor = leds.getColor(index);
        int r = (int) oldColor.red;
        int g = (int) oldColor.green;
        int b = (int) oldColor.blue;
        r = (r <= 10) ? 0 : (int) r - (r * fadeValue / 256);
        g = (g <= 10) ? 0 : (int) g - (g * fadeValue / 256);
        b = (b <= 10) ? 0 : (int) b - (b * fadeValue / 256);
        leds.setRGB(index, r, g, b);
    }
}
