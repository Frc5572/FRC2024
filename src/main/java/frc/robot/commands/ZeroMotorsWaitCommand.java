package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Zeros motors then executes normal wait command.
 */
public class ZeroMotorsWaitCommand extends Command {
    protected Timer m_timer = new Timer();
    private final double m_duration;
    private Swerve s_Swerve;

    /**
     * Zeros motors then executes normal wait command.
     *
     * @param seconds how long the wait command should run
     */
    public ZeroMotorsWaitCommand(double seconds, Swerve swerve) {
        this.m_duration = seconds;
        SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
        this.s_Swerve = swerve;
    }

    @Override
    public void initialize() {
        s_Swerve.setMotorsZero();
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
