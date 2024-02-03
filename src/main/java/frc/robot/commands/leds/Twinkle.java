package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.math.Conversions;
import frc.robot.subsystems.LEDs;

/**
 * Command to move LEDs back and forth like a Cylon eye
 */
public class Twinkle extends Command {
    private final LEDs leds;
    private int numLEDs;
    private int count;
    private Color[] colors = new Color[] {};

    /**
     * Creates a twinkle effect
     *
     * @param leds The LED Subsystem
     * @param count The number of LEDs to twinkle
     * @param colors The colors to randomly select
     */
    public Twinkle(LEDs leds, int count, Color[] colors) {
        this.leds = leds;
        this.numLEDs = leds.getLength();
        this.count = count;
        this.colors = colors;
        addRequirements(leds);
    }

    /**
     * Creates a twinkle effect using random colors
     *
     * @param leds The LED Subsystem
     * @param count The number of LEDs to twinkle
     */
    public Twinkle(LEDs leds, int count) {
        this(leds, count, new Color[] {});
    }

    /**
     * Creates a twinkle effect with 20 LEDs and random colors
     *
     * @param leds The LED Subsystem
     */
    public Twinkle(LEDs leds) {
        this(leds, 20);
    }

    @Override
    public void initialize() {
        leds.setColor(Color.kBlack);
    }

    @Override
    public void execute() {
        leds.setColor(Color.kBlack);
        for (int i = 0; i < count; i++) {
            if (colors.length > 0) {
                leds.setColor(Conversions.random(numLEDs),
                    colors[Conversions.random(colors.length)]);
            } else {
                leds.setRGB(Conversions.random(numLEDs), Conversions.random(0, 255),
                    Conversions.random(0, 255), Conversions.random(0, 255));
            }
            leds.setData();
            Timer.delay(100 / 1000.0);
        }
    }


    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
