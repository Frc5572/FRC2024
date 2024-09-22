package frc.lib.profiling;

/**
 * This logger explicitly does nothing. Because there is no variance in doing nothing, a single
 * instance is provided as {@link #INSTANCE}.
 */
public final class EmptyProfiler implements Profiler {

    /**
     * The sole instance of {@link EmptyProfiler}.
     */
    public static final EmptyProfiler INSTANCE = new EmptyProfiler();

    private EmptyProfiler() {}

    @Override
    public void save() {

    }

    @Override
    public void startTick() {

    }

    @Override
    public void endTick() {

    }

    @Override
    public void push(String location) {

    }

    @Override
    public void pop() {

    }

    @Override
    public void reset() {

    }
}
