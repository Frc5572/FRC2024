package frc.robot.commands.customproxy;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Proxy Parallel Command
 */
public class ProxyParallelCommand extends Command {
    private final Supplier<List<Command>> m_supplier;
    private final Map<Command, Boolean> m_commands = new HashMap<>();


    /**
     * Creates a new ProxyParallelCommand that schedules the supplied command when initialized, and
     * ends when it is no longer scheduled. Useful for lazily creating commands at runtime.
     *
     * @param supplier the command supplier
     */
    public ProxyParallelCommand(Supplier<List<Command>> supplier) {
        m_supplier = supplier;
    }

    // /**
    // * Creates a new ProxyParallelCommand that schedules the given command when initialized, and
    // * ends when it is no longer scheduled.
    // *
    // * @param command the command to run by proxy
    // */
    // @SuppressWarnings("this-escape")
    // public ProxyParallelCommand(Command command) {
    // this(() -> ArraysList<Command>(command));
    // setName("Proxy(" + command.getName() + ")");
    // }

    @Override
    public void initialize() {
        List<Command> m_command = m_supplier.get();
        for (Command command : m_command) {
            m_commands.put(command, true);
            command.schedule();
        }
    }

    @Override
    public final void execute() {
        for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
            if (!commandRunning.getValue()) {
                continue;
            }
            if (!commandRunning.getKey().isScheduled()) {
                commandRunning.setValue(false);
            }
        }
    }

    @Override
    public final void end(boolean interrupted) {
        if (interrupted) {
            for (Map.Entry<Command, Boolean> commandRunning : m_commands.entrySet()) {
                if (commandRunning.getValue()) {
                    commandRunning.getKey().cancel();;
                }
            }
        }
    }

    @Override
    public final boolean isFinished() {
        return !m_commands.containsValue(true);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
