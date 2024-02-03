package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

/**
 * Command to move LEDs back and forth like a Cylon eye
 */
public class BouncingBalls extends Command {
    private final LEDs leds;
    private int ballCount;
    private double gravity = -9.81;
    private int startHeight;
    private double[] height;
    private double impactVelocityStart;
    private double[] impactVelocity;
    private double[] timeSinceLastBounce;
    private int[] position;
    private double[] clocktimeSinceLastBounce;
    private double[] dampening;
    private Color[] colors;

    /**
     * Command to create a bouncing ball effect
     *
     * @param leds The LED Subsystem.
     * @param ballCount The number of balls.
     * @param colors The color of the balls.
     * @param startHeight The height from which to drop the balls
     */
    public BouncingBalls(LEDs leds, int ballCount, Color[] colors, int startHeight) {
        this.leds = leds;
        this.ballCount = ballCount;
        this.startHeight = startHeight;
        this.height = new double[ballCount];
        this.impactVelocityStart = Math.sqrt(-2 * gravity * startHeight);
        this.impactVelocity = new double[ballCount];
        this.timeSinceLastBounce = new double[ballCount];
        this.position = new int[ballCount];
        this.clocktimeSinceLastBounce = new double[ballCount];
        this.dampening = new double[ballCount];
        this.colors = colors;
        addRequirements(leds);
    }

    /**
     * Command to create a bouncing ball effect
     *
     * @param leds The LED Subsystem.
     * @param ballCount The number of balls.
     * @param colors The color of the balls.
     */
    public BouncingBalls(LEDs leds, int ballCount, Color[] colors) {
        this(leds, ballCount, colors, 1);
    }

    /**
     * Command to create a bouncing ball effect
     *
     * @param leds The LED Subsystem.
     * @param ballCount The number of balls.
     */
    public BouncingBalls(LEDs leds, int ballCount) {
        this(leds, ballCount, new Color[] {Color.kRed}, 1);
    }


    /**
     * Command to create a bouncing ball effect
     *
     * @param leds The LED Subsystem.
     */
    public BouncingBalls(LEDs leds) {
        this(leds, 1, new Color[] {Color.kRed}, 1);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < ballCount; i++) {
            clocktimeSinceLastBounce[i] = Timer.getFPGATimestamp();
            height[i] = startHeight;
            position[i] = 0;
            impactVelocity[i] = impactVelocityStart;
            timeSinceLastBounce[i] = 0;
            dampening[i] = 0.90 - i / Math.pow(ballCount, 2);
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < ballCount; i++) {
            timeSinceLastBounce[i] = Timer.getFPGATimestamp() - clocktimeSinceLastBounce[i];
            height[i] = 0.5 * gravity * Math.pow(timeSinceLastBounce[i], 2.0)
                + impactVelocity[i] * timeSinceLastBounce[i];

            if (height[i] < 0) {
                height[i] = 0;
                impactVelocity[i] = dampening[i] * impactVelocity[i];
                clocktimeSinceLastBounce[i] = Timer.getFPGATimestamp();

                if (impactVelocity[i] < 0.01) {
                    impactVelocity[i] = impactVelocityStart;
                }
            }
            position[i] = (int) Math.round(height[i] * (leds.getLength() - 1) / startHeight);
        }
        for (int i = 0; i < leds.getLength(); i++) {
            leds.setColor(Color.kBlack);
        }
        for (int i = 0; i < ballCount; i++) {
            Color c = i >= colors.length ? Color.kRed : colors[i];
            leds.setColor(position[i], c);
        }
        leds.setData();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
