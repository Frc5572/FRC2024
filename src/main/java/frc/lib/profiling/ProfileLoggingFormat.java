package frc.lib.profiling;

import java.io.OutputStream;
import frc.lib.profiling.LoggingProfiler.LocatedInfo;

/**
 * Describes how a profile is written to a file.
 */
public interface ProfileLoggingFormat {

    /**
     * Comparison function used for sorting profile information.
     */
    int compare(String leftName, LocatedInfo leftInfo, String rightName, LocatedInfo rightInfo);

    /**
     * Write a profile to the file specified in {@code outStream}.
     */
    void write(String name, LocatedInfo info, double timeDivisor, OutputStream outStream);

    /**
     * Write information before seeing any profiles.
     */
    void begin(OutputStream outputStream);

    /**
     * Write information after seeing all profiles.
     */
    void end(OutputStream outputStream);

}
