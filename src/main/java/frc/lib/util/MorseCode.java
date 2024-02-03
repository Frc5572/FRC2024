package frc.lib.util;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A library for using Morse Code
 */
public class MorseCode {

    public static JsonNode morseJSON =
        openJson(new File(Filesystem.getDeployDirectory(), "morse-code.json"));

    /**
     * Open JSON file.
     *
     * @param file JSON File to open.
     * @return JsonNode of file.
     */
    private static JsonNode openJson(File file) {
        try {
            return new ObjectMapper().readTree(file);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Generate a Map of dots and dashes for each character in the Morse Code Alphabet
     *
     * @param unit The length of a unit
     * @return A map of characters to dot-dash codes
     */
    public static Map<String, ArrayList<Integer>> generateMorseCodeAlphabet(int unit) {
        int dot = unit;
        int dash = unit * 3;
        int word_space = unit * 7;
        HashMap<String, ArrayList<Integer>> letters = new HashMap<String, ArrayList<Integer>>();
        Iterator<Entry<String, JsonNode>> iterator = morseJSON.fields();
        iterator.forEachRemaining(field -> {
            String key = field.getKey();
            char[] parts = field.getValue().textValue().toCharArray();
            ArrayList<Integer> arr = new ArrayList<Integer>();
            for (char x : parts) {
                if ('.' == x) {
                    arr.add(dot);
                } else if ('-' == x) {
                    arr.add(dash);
                }
            }
            letters.put(key, arr);
        });
        letters.put(" ", new ArrayList<Integer>(List.of(word_space)));
        return letters;
    }

    /**
     * Generate an array of True or False values to indicate On or Off.
     *
     * @param word The Word or Sentence to convert to Morse Code
     * @param unit The base unit/length of time for the conversion
     * @return An array of True/False
     */
    public static ArrayList<Boolean> generateDitArray(String word, int unit) {
        Map<String, ArrayList<Integer>> letters = generateMorseCodeAlphabet(unit);
        int inter_character = unit * 3;
        int word_space = unit * 7;
        ArrayList<Boolean> finalWord = new ArrayList<Boolean>();
        word = word.toLowerCase();
        String[] parts = word.split(" ");
        for (int a = 0; a < parts.length; a++) {
            char[] ch = parts[a].toCharArray();
            for (int x = 0; x < ch.length; x++) {
                for (int y = 0; y < letters.get(String.valueOf(ch[x])).size(); y++) {
                    for (int z = 0; z < letters.get(String.valueOf(ch[x])).get(y); z++) {
                        finalWord.add(true);
                    }
                    if (y < letters.get(String.valueOf(ch[x])).size() - 1) {
                        Boolean[] q = new Boolean[unit];
                        Arrays.fill(q, false);
                        finalWord.addAll(Arrays.asList(q));
                    }
                }
                if (x < ch.length - 1) {
                    Boolean[] q = new Boolean[inter_character];
                    Arrays.fill(q, false);
                    finalWord.addAll(Arrays.asList(q));
                }
            }
            if (a < parts.length - 1) {
                Boolean[] q = new Boolean[word_space];
                Arrays.fill(q, false);
                finalWord.addAll(Arrays.asList(q));
            }
        }
        return finalWord;
    }

    /**
     * Generates an array of color objects to pass to the LED controller.
     *
     * @param finalWord An array of True/False values.
     * @param primaryColor The color to use when the array value is.
     * @return An array of color objects.
     */
    public static ArrayList<Color> generateDirColorArray(ArrayList<Boolean> finalWord,
        Color primaryColor) {
        ArrayList<Color> finalStrip = new ArrayList<Color>();
        for (int c = 0; c < finalWord.size(); c++) {
            boolean x = finalWord.get(c);
            Color color = x ? primaryColor : Color.kBlack;
            finalStrip.add(color);
        }
        return finalStrip;
    }

    /**
     * Generates an array of color objects to pass to the LED controller.
     *
     * @param word The Word or Sentence to convert to Morse Code
     * @param unit The base unit/length of time for the conversion
     * @param primaryColor The color to use when the array value is.
     * @return An array of color objects.
     */
    public static ArrayList<Color> generateDirColorArray(String word, int unit,
        Color primaryColor) {
        ArrayList<Boolean> finalWord = generateDitArray(word, unit);
        return generateDirColorArray(finalWord, primaryColor);
    }

}
