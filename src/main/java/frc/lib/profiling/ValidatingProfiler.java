package frc.lib.profiling;

/**
 * This logger only performs validation (i.e. throws if {@link #pop() pop} is called with an empty
 * stack). Otherwise, does nothing. Because there is no variance in doing nothing, a single instance
 * is provided as {@link #INSTANCE}.
 */
public final class ValidatingProfiler implements Profiler {

    /**
     * The sole instance of {@link ValidatingProfiler}.
     */
    public static final ValidatingProfiler INSTANCE = new ValidatingProfiler();

    private ValidatingProfiler() {}

    private boolean tickStarted = false;
    private int pathLen = 0;
    private boolean fullPathEmpty = true;

    @Override
    public void save() {

    }

    @Override
    public void startTick() {
        if (this.tickStarted) {
            throw new RuntimeException("Profiler tick already started. Missing endTick()?");
        } else {
            tickStarted = true;
            pathLen = 0;
            fullPathEmpty = true;
            this.push("root");
        }
    }

    @Override
    public void endTick() {
        if (!this.tickStarted) {
            throw new RuntimeException("Profiler tick already ended. Missing startTick()?");
        } else {
            this.pop();
            this.tickStarted = false;
            if (!this.fullPathEmpty) {
                throw new RuntimeException(
                    "Profiler tick ended before path was fully popped. Mismatched push/pop?");
            }
        }
    }

    @Override
    public void push(String location) {
        if (!this.tickStarted) {
            throw new RuntimeException("Cannot push '" + location
                + "' to the profiler if profiler tick hasn't started. Missing startTick()?");
        } else {
            fullPathEmpty = false;
            pathLen += 1;
        }
    }

    @Override
    public void pop() {
        if (!this.tickStarted) {
            throw new RuntimeException(
                "Cannot pop from profiler if profiler tick hasn't started. Missing startTick()?");
        } else if (pathLen == 0) {
            throw new RuntimeException(
                "Tried to pop one too many times! Mismatched push() and pop()?");
        } else {
            pathLen -= 1;
            if (pathLen == 0) {
                fullPathEmpty = true;
            }
        }
    }
}
