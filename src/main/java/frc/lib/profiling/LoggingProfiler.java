package frc.lib.profiling;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.LongSupplier;
import org.littletonrobotics.junction.Logger;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
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
    private boolean tickStarted;
    private String fullPath = "";
    private LoggingProfiler.LocatedInfo currentInfo;

    /**
     * @param timeGetter a supplier for the current time.
     * @param timeDivisor a conversion factor turning the units of {@code timeGetter} to the units
     *        output to a file.
     */
    public LoggingProfiler(LongSupplier timeGetter, double timeDivisor) {
        this.timeGetter = timeGetter;
        this.timeDivisor = timeDivisor;
    }

    @Override
    public void save() {
        long start = timeGetter.getAsLong();
        HashMap<String, LocatedInfoJSON> jsons = new HashMap<>();
        for (var entry : locationInfos.entrySet()) {
            var value = new LocatedInfoJSON();
            value.visits = entry.getValue().visits;
            value.maxTime = entry.getValue().maxTime;
            value.minTime = entry.getValue().minTime;
            value.totalTime = entry.getValue().totalTime;
            value.children = new HashMap<>();
            jsons.put(entry.getKey(), value);
        }

        System.out.println("----");
        for (var entry : jsons.entrySet()) {
            System.out.println(entry.getKey());
        }
        System.out.println("----");

        for (var entry : jsons.entrySet()) {
            String key = entry.getKey();
            String[] parts = key.split("\\" + SPLIT_CHAR);
            if (parts.length < 2) {
                continue;
            }
            StringBuilder parent = new StringBuilder();
            for (int i = 0; i < parts.length - 2; i++) {
                parent.append(parts[i]);
                parent.append(SPLIT_CHAR);
            }
            parent.append(parts[parts.length - 2]);
            System.out.println(parent.toString());
            jsons.get(parent.toString()).children.put(parts[parts.length - 1], entry.getValue());
        }

        // Write to file.
        try {
            ByteArrayOutputStream outStream = new ByteArrayOutputStream();
            LocatedInfoJSON root = jsons.get("root");
            JsonFactory factory = new JsonFactory();
            JsonGenerator generator = factory.createGenerator(outStream);
            root.writeJSON(generator, timeDivisor, 0, root.totalTime);
            generator.flush();
            generator.close();
            Logger.recordOutput("profile/json", new String(outStream.toByteArray()));
        } catch (IOException e) {
            e.printStackTrace();
        }
        long end = timeGetter.getAsLong();
        double timeDiff = (end - start) / timeDivisor;
        Logger.recordOutput("profile/timeToWrite", timeDiff);
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

    /**
     * JSON Tree version of {@link LocatedInfo}
     */
    private static class LocatedInfoJSON {
        long maxTime = Long.MIN_VALUE;
        long minTime = Long.MAX_VALUE;
        long totalTime;
        long visits;
        HashMap<String, LocatedInfoJSON> children;

        void writeJSON(JsonGenerator generator, double timeDivisor, long parent_total,
            long root_total) throws IOException {
            generator.writeStartObject();
            generator.writeNumberField("visitCount", visits);
            generator.writeNumberField("totalTime", totalTime / timeDivisor);
            if (parent_total > 0) {
                double percent = (double) totalTime / (double) parent_total * 100.0;
                generator.writeNumberField("percentOfParent", percent);
            }
            if (root_total > 0) {
                double percent = (double) totalTime / (double) root_total * 100.0;
                generator.writeNumberField("percentOfRoot", percent);
            }
            generator.writeNumberField("maxTime", maxTime / timeDivisor);
            generator.writeNumberField("minTime", minTime / timeDivisor);
            generator.writeNumberField("avgTime", totalTime / timeDivisor / visits);
            generator.writeFieldName("children");
            generator.writeStartObject();
            for (var entry : children.entrySet()) {
                generator.writeFieldName(entry.getKey());
                entry.getValue().writeJSON(generator, timeDivisor, totalTime, root_total);
            }
            generator.writeEndObject();
            generator.writeEndObject();
        }
    }

    @Override
    public void reset() {
        this.locationInfos.clear();
        this.fullPath = "";
        this.path.clear();
        this.timeList.clear();
    }

}
