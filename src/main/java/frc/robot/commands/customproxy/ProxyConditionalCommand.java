package frc.robot.commands.customproxy;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Proxy Conditional Command
 */
public class ProxyConditionalCommand extends Command {
    private final Command m_onTrue;
    private final Command m_onFalse;
    private final BooleanSupplier m_condition;
    private Command m_selectedCommand;

    /**
     * Creates a new ProxyConditionalCommand.
     *
     * @param onTrue the command to run if the condition is true
     * @param onFalse the command to run if the condition is false
     * @param condition the condition to determine which command to run
     */
    public ProxyConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        m_onTrue = requireNonNullParam(onTrue, "onTrue", "ProxyConditionalCommand");
        m_onFalse = requireNonNullParam(onFalse, "onFalse", "ProxyConditionalCommand");
        m_condition = requireNonNullParam(condition, "condition", "ProxyConditionalCommand");

        CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);

    }

    @Override
    public void initialize() {
        if (m_condition.getAsBoolean()) {
            m_selectedCommand = m_onTrue;
        } else {
            m_selectedCommand = m_onFalse;
        }
        m_selectedCommand.schedule();
    }

    @Override
    public final void execute() {}

    @Override
    public final void end(boolean interrupted) {
        if (interrupted) {
            m_selectedCommand.cancel();
        }
    }

    @Override
    public final boolean isFinished() {
        return !m_selectedCommand.isScheduled();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
