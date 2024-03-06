package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is the class header for the LEDs Subsystem
 */
public class LEDs extends SubsystemBase {
    private AddressableLEDBuffer controLedBuffer;
    @SuppressWarnings("IOCheck")
    private AddressableLED addressableLED;

    /**
     * constructs a LED Subsystem
     *
     * @param length length of the addressable LEDS
     * @param port port ID for PWM
     */
    public LEDs(int length, int port) {
        controLedBuffer = new AddressableLEDBuffer(length);
        addressableLED = new AddressableLED(port);

        addressableLED.setLength(controLedBuffer.getLength());
        addressableLED.setData(controLedBuffer);
        addressableLED.start();
    }

    /**
     * Get LED strip length
     *
     * @return number of LEDs
     */
    public int getLength() {
        return controLedBuffer.getLength();
    }

    /**
     * Set individual LED color via HSV
     *
     * @param index the LED index to set
     * @param h the h value [0-180)
     * @param s the s value [0-255]
     * @param v the v value [0-255]
     */
    public void setHSV(int index, int h, int s, int v) {
        controLedBuffer.setHSV(index, h, s, v);
    }

    /**
     * Sets the LED output data.
     */
    public void setData() {
        addressableLED.setData(controLedBuffer);
    }

    /**
     * Set individual LED color via RGB
     *
     * @param index the LED index to set
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    public void setRGB(int index, int r, int g, int b) {
        controLedBuffer.setRGB(index, r, g, b);
    }

    /**
     * Sets RGB Color of the entire LED strip
     *
     * @param r - [0 - 255]
     * @param g - [0 - 255]
     * @param b - [0 - 255]
     */
    public void setRGB(int r, int g, int b) {
        for (var i = 0; i < getLength(); i++) {
            setRGB(i, r, g, b);
        }
        setData();
    }

    /**
     * Set individual LED color via Color
     *
     * @param index the LED index to set
     * @param color The color of the LED
     */
    public void setColor(int index, Color color) {
        controLedBuffer.setLED(index, color);
    }

    /**
     * Sets the Color of the entire LED strip
     *
     * @param color color set for the LEDs
     */
    public void setColor(Color color) {
        for (var i = 0; i < getLength(); i++) {
            setColor(i, color);
        }
        setData();
    }

    public Color getColor(int index) {
        return controLedBuffer.getLED(index);
    }
}
