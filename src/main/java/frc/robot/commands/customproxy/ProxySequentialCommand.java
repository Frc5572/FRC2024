package frc.robot.commands.customproxy;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Proxy Sequential Command
 */
public class ProxySequentialCommand extends Command {
    private final Supplier<List<Command>> m_supplier;
    private final List<Command> m_commands = new ArrayList<>();
    private int m_currentCommandIndex = -1;


    /**
     * Creates a new ProxySequentialCommand that schedules the supplied command when initialized,
     * and ends when it is no longer scheduled. Useful for lazily creating commands at runtime.
     *
     * @param supplier the command supplier
     */
    public ProxySequentialCommand(Supplier<List<Command>> supplier) {
        m_supplier = supplier;
    }

    // /**
    // * Creates a new ProxySequentialCommand that schedules the given command when initialized, and
    // * ends when it is no longer scheduled.
    // *
    // * @param command the command to run by proxy
    // */
    // @SuppressWarnings("this-escape")
    // public ProxySequentialCommand(Command command) {
    // this(() -> ArraysList<Command>(command));
    // setName("Proxy(" + command.getName() + ")");
    // }

    @Override
    public void initialize() {
        m_currentCommandIndex = 0;
        List<Command> m_command = m_supplier.get();
        for (Command command : m_command) {
            m_commands.add(command);
        }
        if (!m_commands.isEmpty()) {
            m_commands.get(0).schedule();
        }
    }

    @Override
    public final void execute() {
        if (m_commands.isEmpty()) {
            return;
        }
        Command currentCommand = m_commands.get(m_currentCommandIndex);
        if (!currentCommand.isScheduled()) {
            m_currentCommandIndex++;
        }
        if (m_currentCommandIndex < m_commands.size()) {
            m_commands.get(m_currentCommandIndex).schedule();
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted && !m_commands.isEmpty() && m_currentCommandIndex > -1
            && m_currentCommandIndex < m_commands.size()) {
            m_commands.get(m_currentCommandIndex).cancel();
        }
        m_currentCommandIndex = -1;
    }

    @Override
    public final boolean isFinished() {
        return m_currentCommandIndex == m_commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
