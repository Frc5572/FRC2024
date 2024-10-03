package frc.lib.profiling;

import java.util.HashMap;
import java.util.Map;

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

    /**
     * Keeps track of methods and how many times they call push/pop.
     */
    private Map<String, Integer> methodPushHelper = new HashMap<>();

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
            StringBuilder builder = new StringBuilder();
            for (var entry : methodPushHelper.entrySet()) {
                if (entry.getValue() != 0) {
                    builder.append(entry.getKey());
                    builder.append(" has ");
                    builder.append(entry.getValue());
                    builder.append(" more pushes that pops.\n");
                }
            }
            throw new RuntimeException(
                "Profiler tick already ended. Missing startTick()?\n" + builder.toString());
        } else {
            this.pop();
            this.tickStarted = false;
            if (!this.fullPathEmpty) {
                StringBuilder builder = new StringBuilder();
                for (var entry : methodPushHelper.entrySet()) {
                    if (entry.getValue() != 0) {
                        builder.append(entry.getKey());
                        builder.append(" has ");
                        builder.append(entry.getValue());
                        builder.append(" more pushes that pops.\n");
                    }
                }
                throw new RuntimeException(
                    "Profiler tick ended before path was fully popped. Mismatched push/pop?\n"
                        + builder.toString());
            }
        }
    }

    @Override
    public void push(String location) {
        if (!this.tickStarted) {
            throw new RuntimeException("Cannot push '" + location
                + "' to the profiler if profiler tick hasn't started. Missing startTick()?");
        } else {
            for (var stack : Thread.currentThread().getStackTrace()) {
                if (stack.getClassName().equals(ValidatingProfiler.class.getCanonicalName())
                    || stack.getClassName().equals(Profiler.class.getCanonicalName())
                    || stack.getClassName().equals(Thread.class.getCanonicalName())) {
                    continue;
                }
                String key = stack.getClassName() + "." + stack.getMethodName();
                methodPushHelper.put(key, methodPushHelper.computeIfAbsent(key, (_k) -> 0) + 1);
                break;
            }
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
            StringBuilder builder = new StringBuilder();
            for (var entry : methodPushHelper.entrySet()) {
                if (entry.getValue() != 0) {
                    builder.append(entry.getKey());
                    builder.append(" has ");
                    builder.append(-entry.getValue());
                    builder.append(" more pops than pushes.\n");
                }
            }
            throw new RuntimeException(
                "Tried to pop one too many times! Mismatched push() and pop()?\n"
                    + builder.toString());
        } else {
            for (var stack : Thread.currentThread().getStackTrace()) {
                if (stack.getClassName().equals(ValidatingProfiler.class.getCanonicalName())
                    || stack.getClassName().equals(Profiler.class.getCanonicalName())
                    || stack.getClassName().equals(Thread.class.getCanonicalName())) {
                    continue;
                }
                String key = stack.getClassName() + "." + stack.getMethodName();
                methodPushHelper.put(key, methodPushHelper.computeIfAbsent(key, (_k) -> 0) - 1);
                break;
            }
            pathLen -= 1;
            if (pathLen == 0) {
                fullPathEmpty = true;
            }
        }
    }

    @Override
    public void reset() {

    }
}
