package frc.robot.commands.leds;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.MorseCode;
import frc.robot.subsystems.LEDs;

/**
 * Command to flash the LED strip between 2 colors
 *
 * <p>
 * DOES NOT WORK YET
 */
public class MorseCodeStream extends Command {
    private LEDs leds;
    private int index = 0;
    private ArrayList<Boolean> finalWord = new ArrayList<Boolean>();
    private ArrayList<Color> finalStrip = new ArrayList<Color>();

    /**
     * Command to flash the LED strip between 2 colors
     *
     * @param leds LED Subsystem
     * @param word The word or sentence to flash in Morse Code
     * @param color The color to flash
     */
    public MorseCodeStream(LEDs leds, String word, Color color) {
        this.leds = leds;
        finalWord = MorseCode.generateDitArray(word, 1);
        finalStrip = MorseCode.generateDirColorArray(finalWord, color);
        addRequirements(leds);
    }

    /**
     * Command to flash the LED strip between 2 colors
     *
     * @param leds LED Subsystem
     * @param word The word or sentence to flash in Morse Code
     */
    public MorseCodeStream(LEDs leds, String word) {
        this(leds, word, Color.kRed);
    }

    @Override
    public void initialize() {
        index = 0;
        leds.setColor(Color.kBlack);
        System.out.println(finalStrip);
    }


    @Override
    public void execute() {
        leds.setColor(Color.kBlack);
        System.out.println("Index: " + index);
        for (int i = index; i >= 0; i--) {
            System.out.println("I: " + i);
            if (index < leds.getLength()) {
                int q = index - i;
                if (q < finalStrip.size()) {
                    leds.setColor(i, finalStrip.get(q));
                }
            } else if (index >= leds.getLength()) {
                int q = index - (leds.getLength() - 1);
                System.out.println("Q: " + q);
                int k = index - i;
                int w = q + k;
                System.out.println("K: " + k);
                System.out.println("W: " + w);
                if (w >= 0 && w < finalStrip.size()) {
                    leds.setColor(leds.getLength() - q, finalStrip.get(w));
                }
            }
        }
        leds.setData();
        index++;
        Timer.delay(.2);
    }

    @Override
    public boolean isFinished() {
        return false; // index >= finalStrip.size() + leds.getLength() - 1;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

