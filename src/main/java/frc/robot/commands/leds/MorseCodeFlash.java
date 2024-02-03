package frc.robot.commands.leds;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.MorseCode;
import frc.robot.subsystems.LEDs;

/**
 * Command to flash the LED strip between 2 colors
 */
public class MorseCodeFlash extends Command {
    private LEDs leds;
    private int index = 0;
    private ArrayList<Boolean> finalWord = new ArrayList<Boolean>();
    private ArrayList<Color> finalStrip;

    /**
     * Command to flash the LED strip between 2 colors
     *
     * @param leds LED Subsystem
     * @param word The word or sentence to flash in Morse Code
     * @param color The color to flash
     */
    public MorseCodeFlash(LEDs leds, String word, Color color) {
        this.leds = leds;
        addRequirements(leds);
        finalWord = MorseCode.generateDitArray(word, 10);
        finalStrip = MorseCode.generateDirColorArray(finalWord, color);
    }

    /**
     * Command to flash the LED strip between 2 colors
     *
     * @param leds LED Subsystem
     * @param word The word or sentence to flash in Morse Code
     */
    public MorseCodeFlash(LEDs leds, String word) {
        this(leds, word, Color.kRed);
    }

    @Override
    public void initialize() {
        index = 0;
        leds.setColor(Color.kBlack);
    }


    @Override
    public void execute() {
        Color x = finalStrip.get(index);
        leds.setColor(x);
        index++;
    }

    @Override
    public boolean isFinished() {
        return index >= finalStrip.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}

