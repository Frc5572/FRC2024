package frc.lib.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import java.util.Map;
import java.util.function.Supplier;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;

/**
 * Similar to {@link SelectCommand} but allows commands for all variants of an enum instead. Also
 * doesn't require all subsystems of all commands, only the selected command on run time.
 */
public class LazySelectCommand<K> extends Command {
    private final Map<K, Command> m_commands;
    private final Supplier<? extends K> m_selector;
    private Command m_command;

    /**
     * Create new LazySelectCommand. Note that the length of enumValues and commandValues must be
     * the same.
     *
     * @throws IllegalArgumentException when enumValues and commandValues have different lengths.
     */
    public LazySelectCommand(Map<K, Command> commands, Supplier<? extends K> selector) {
        m_commands = requireNonNullParam(commands, "commands", "LazySelectCommand");
        m_selector = requireNonNullParam(selector, "selector", "LazySelectCommand");
    }

    @Override
    public void initialize() {
        m_command = m_commands.getOrDefault(m_selector.get(), Commands.none());
        m_command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_command.cancel();
        }
        m_command = null;
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        // because we're between `initialize` and `end`, `m_command` is necessarily not null
        // but if called otherwise and m_command is null,
        // it's UB, so we can do whatever we want -- like return true.
        return m_command == null || !m_command.isScheduled();
    }

    /**
     * Whether the given command should run when the robot is disabled. Override to return true if
     * the command should run when disabled.
     *
     * @return true. Otherwise, this proxy would cancel commands that do run when disabled.
     */
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("lazy select command",
            () -> m_command == null ? "null" : m_command.getName(), null);
    }
}
