package frc.lib.profiling;

import java.io.IOException;
import java.io.OutputStream;
import frc.lib.profiling.LoggingProfiler.LocatedInfo;

/**
 * Logging format that writes to a human-readable text file. Profile sections are sorted from by
 * longest total time (from most time to least time). Because there is no variance, a single
 * instance is provided as {@link #INSTANCE}.
 */
public class TextProfileLoggingFormat implements ProfileLoggingFormat {
    private TextProfileLoggingFormat() {}

    /**
     * The sole instance of {@link TextProfileLoggingFormat}.
     */
    public static final TextProfileLoggingFormat INSTANCE = new TextProfileLoggingFormat();

    @Override
    public int compare(String leftName, LocatedInfo leftInfo, String rightName,
        LocatedInfo rightInfo) {
        return -Long.compare(leftInfo.totalTime, rightInfo.totalTime);
    }

    @Override
    public void write(String name, LocatedInfo info, double timeDivisor, OutputStream outStream) {
        try {
            outStream.write((name + "\n").getBytes());
            outStream.write(("    visitCount: " + info.visits + "\n").getBytes());
            outStream.write(("    totalTime: " + info.totalTime / timeDivisor + "\n").getBytes());
            outStream.write(("    maxTime: " + info.maxTime / timeDivisor + "\n").getBytes());
            outStream.write(("    minTime: " + info.minTime / timeDivisor + "\n").getBytes());
            outStream.write(
                ("    avgTime: " + info.totalTime / info.visits / timeDivisor + "\n").getBytes());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void begin(OutputStream outputStream) {
        try {
            outputStream.write(("Performance Log\n" + //
                "================================================================================\n")
                    .getBytes());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void end(OutputStream outputStream) {

    }

}
