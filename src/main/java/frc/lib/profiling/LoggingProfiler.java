package frc.lib.profiling;

import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.Field;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.ReceiverThread;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.lib.util.LongArrayList;

/**
 * This logger saves runtime performance metrics to memory. These metrics are subsequently written
 * to a file when {@link #save() save} is called.
 */
public final class LoggingProfiler implements Profiler {
    private static final char SPLIT_CHAR = '\u001e';

    private final List<String> path = new ArrayList<>();
    private final LongArrayList timeList = new LongArrayList();
    private final Map<String, LoggingProfiler.LocatedInfo> locationInfos = new HashMap<>();
    private final LongSupplier timeGetter;
    private final double timeDivisor;
    private final ProfileLoggingFormat loggingFormat;
    private boolean tickStarted;
    private String fullPath = "";
    private LoggingProfiler.LocatedInfo currentInfo;

    /**
     * @param timeGetter a supplier for the current time.
     * @param timeDivisor a conversion factor turning the units of {@code timeGetter} to the units
     *        output to a file.
     * @param loggingFormat file format to use when {@link #save() save} is called.
     */
    public LoggingProfiler(LongSupplier timeGetter, double timeDivisor,
        ProfileLoggingFormat loggingFormat) {
        this.timeGetter = timeGetter;
        this.timeDivisor = timeDivisor;
        this.loggingFormat = loggingFormat;
    }

    @Override
    public void save() {
        try {
            // Determine the filepath of the current logger, except the suffix is "-profile.txt"
            // instead of ".wpilog".
            String filePath = null;

            Field dataReceiversField = ReceiverThread.class.getDeclaredField("dataReceivers");
            dataReceiversField.setAccessible(true);
            Field receiverThreadField = Logger.class.getDeclaredField("receiverThread");
            receiverThreadField.setAccessible(true);
            Field logDateField = WPILOGWriter.class.getDeclaredField("logDate");
            logDateField.setAccessible(true);
            Field randomIdentifierField = WPILOGWriter.class.getDeclaredField("randomIdentifier");
            randomIdentifierField.setAccessible(true);
            Field timeFormatterField = WPILOGWriter.class.getDeclaredField("timeFormatter");
            timeFormatterField.setAccessible(true);
            Field logMatchTextField = WPILOGWriter.class.getDeclaredField("logMatchText");
            logMatchTextField.setAccessible(true);

            Object dataReceivers = dataReceiversField.get(receiverThreadField.get(null));
            if (dataReceivers instanceof List<?> dataReceiversList) {
                for (Object dataReceiver : dataReceiversList) {
                    if (dataReceiver instanceof WPILOGWriter dataWriter) {
                        StringBuilder newFilenameBuilder = new StringBuilder();
                        newFilenameBuilder.append("Log_");
                        LocalDateTime logDate = (LocalDateTime) logDateField.get(dataWriter);
                        if (logDate == null) {
                            Object randomIdentifier = randomIdentifierField.get(dataWriter);
                            newFilenameBuilder.append(randomIdentifier);
                        } else {
                            DateTimeFormatter timeFormatter =
                                (DateTimeFormatter) timeFormatterField.get(dataWriter);
                            newFilenameBuilder.append(timeFormatter.format(logDate));
                        }
                        Object logMatchText = logMatchTextField.get(dataWriter);
                        if (logMatchText != null) {
                            newFilenameBuilder.append("_");
                            newFilenameBuilder.append(logMatchText);
                        }
                        newFilenameBuilder.append("-profile.txt");
                        filePath = newFilenameBuilder.toString();
                    }
                }
            }

            // Write to file.
            try (var outStream = new FileOutputStream(filePath)) {
                loggingFormat.begin(outStream);
                for (var entry : locationInfos.entrySet().stream().sorted((a, b) -> loggingFormat
                    .compare(a.getKey(), a.getValue(), b.getKey(), b.getValue())).toList()) {
                    var info = entry.getValue();
                    var name = entry.getKey().replace(SPLIT_CHAR, '.');
                    loggingFormat.write(name, info, timeDivisor, outStream);
                }
                loggingFormat.end(outStream);
            }
        } catch (IOException | NoSuchFieldException | SecurityException | IllegalArgumentException
            | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void startTick() {
        if (this.tickStarted) {
            throw new RuntimeException("Profiler tick already started. Missing endTick()?");
        } else {
            this.tickStarted = true;
            this.fullPath = "";
            this.path.clear();
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
            if (!this.fullPath.isEmpty()) {
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
            if (!this.fullPath.isEmpty()) {
                this.fullPath = this.fullPath + SPLIT_CHAR;
            }
            this.fullPath += location;
            this.path.add(this.fullPath);
            this.timeList.add(timeGetter.getAsLong());
            this.currentInfo = null;
        }
    }

    @Override
    public void pop() {
        if (!this.tickStarted) {
            throw new RuntimeException(
                "Cannot pop from profiler if profiler tick hasn't started. Missing startTick()?");
        } else if (this.timeList.isEmpty()) {
            throw new RuntimeException(
                "Tried to pop one too many times! Mismatched push() and pop()?");
        } else {
            this.path.remove(this.path.size() - 1);

            long currentTime = timeGetter.getAsLong();
            long startTime = this.timeList.remove(this.timeList.size() - 1);
            long timeSpan = currentTime - startTime;

            LocatedInfo locatedInfo = this.getCurrentInfo();

            locatedInfo.totalTime += timeSpan;
            locatedInfo.visits++;
            locatedInfo.maxTime = Math.max(locatedInfo.maxTime, timeSpan);
            locatedInfo.minTime = Math.min(locatedInfo.minTime, timeSpan);
            this.fullPath = this.path.isEmpty() ? "" : this.path.get(this.path.size() - 1);
            this.currentInfo = null;
        }
    }

    private LocatedInfo getCurrentInfo() {
        if (this.currentInfo == null) {
            this.currentInfo =
                this.locationInfos.computeIfAbsent(this.fullPath, _k -> new LocatedInfo());
        }
        return this.currentInfo;
    }

    /**
     * Performance metrics for a given state of the profile stack.
     */
    public static class LocatedInfo {
        long maxTime = Long.MIN_VALUE;
        long minTime = Long.MAX_VALUE;
        long totalTime;
        long visits;
    }

}
