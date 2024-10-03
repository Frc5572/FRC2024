package frc.lib.profiling;

/**
 * Interface supporting profiling calls for measuring and saving runtime performance.
 *
 * <p>
 * Operates on the idea of a "profile stack", which is just a stack of names with associated
 * profiling metrics. For instance, the following code
 *
 * <pre>
 * {@code
 * Profiler profiler = someProfiler();
 * profiler.startTick();
 * profiler.push("Hello")
 * someExpensiveFunction();
 * profiler.swap("World")
 * someLessExpensiveFunction();
 * profiler.pop();
 * profiler.endTick();
 * }
 * </pre>
 *
 * <p>
 * will produce a profile with three sections:
 * <ul>
 * <li>{@code root}, which describes the time between {@link #startTick() profiler.startTick()} and
 * {@link #endTick() profiler.endTick()},</li>
 * <li>{@code root.Hello}, which describes the time between {@link #push(String)
 * profiler.push("Hello")} and {@link #swap(String) profiler.swap("World")}, and</li>
 * <li>{@code root.World}, which describes the time between {@link #swap(String)
 * profiler.swap("World")} and {@link #pop() profiler.pop()}.
 * </ul>
 *
 * <p>
 * A call to {@link #save() profiler.save()} would output the performance metrics of these sections
 * in the case of a {@link LoggingProfiler}.
 */
public sealed interface Profiler permits EmptyProfiler, LoggingProfiler, ValidatingProfiler {
    /**
     * Called at the top of the main loop. Indicates the profiler is at "root" and records the start
     * time. Within the main loop, no meaningful work should occur before this call.
     *
     * @throws RuntimeException if called twice without a call to {@link #endTick() endTick} in
     *         between.
     */
    void startTick();

    /**
     * Called at the bottom of the main loop. Indicates the profiler's "root" is finished, and its
     * time usage metrics are updated. Within the main loop, no meaningful work should occur after
     * this call.
     *
     * @throws RuntimeException if the profile stack is not empty or {@link #startTick() startTick}
     *         has not been called.
     */
    void endTick();

    /**
     * The profile stack is pushed with the {@code location} as its top. Must be accompanied by a
     * call to {@link #pop() pop}.
     *
     * @throws RuntimeException if {@link #startTick() startTick} hasn't been called yet.
     */
    void push(String location);

    /**
     * The profile stack is popped. Must be preceded (at some point) by a call to
     * {@link #push(String) push}.
     *
     * @throws RuntimeException if {@link #startTick() startTick} hasn't been called yet or the
     *         profile stack is empty.
     */
    void pop();

    /**
     * The top of the profile stack is replaced with {@code location}. This is equivalent to a call
     * to {@link #pop() pop} followed immediately by a call to {@link #push(String) push}.
     *
     * @throws RuntimeException if the profile stack is empty.
     */
    default void swap(String location) {
        pop();
        push(location);
    }

    /**
     * Write profile data to a file.
     */
    void save();

    /**
     * Reset logger.
     */
    void reset();
}
