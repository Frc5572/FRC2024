package frc.lib.util;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Similar to {@link ConditionalCommand} but allows commands for all variants of an enum instead.
 */
public class MatchCommand<T extends Enum<T>> extends Command {

    private final Map<T, Command> map;
    private final Supplier<T> condition;
    private Command currentCommand;

    /**
     * Create new MatchCommand. Note that the length of enumValues and commandValues must be the
     * same.
     *
     * @throws IllegalArgumentException when enumValues and commandValues have different lengths.
     */
    public MatchCommand(List<T> enumValues, List<Command> commandValues, Supplier<T> condition) {
        this.map = new HashMap<>();
        this.condition = condition;
        if (enumValues.size() != commandValues.size()) {
            throw new IllegalArgumentException(
                "enumValues.size() should equal commandValues.size(). It does not.");
        }
        for (int i = 0; i < enumValues.size(); i++) {
            this.map.put(enumValues.get(i), commandValues.get(i));
        }
    }

    @Override
    public void initialize() {
        T t = condition.get();
        currentCommand = map.get(t);
        if (currentCommand != null) {
            currentCommand.initialize();
            for (Subsystem s : currentCommand.getRequirements()) {
                this.m_requirements.add(s);
            }
        }
    }

    @Override
    public void execute() {
        if (currentCommand != null) {
            currentCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        currentCommand.end(interrupted);
        for (Subsystem s : currentCommand.getRequirements()) {
            this.m_requirements.remove(s);
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommand == null || currentCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return map.values().stream().allMatch((x) -> x.runsWhenDisabled());
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (map.values().stream()
            .anyMatch((x) -> x.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf)) {
            return InterruptionBehavior.kCancelSelf;
        } else {
            return InterruptionBehavior.kCancelIncoming;
        }
    }

}
