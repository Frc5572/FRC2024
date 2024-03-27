package frc.lib.util.rospf;

import java.util.ArrayList;
import edu.wpi.first.math.Pair;

public class VisionMeasurement {

    private int id;
    private final ArrayList<Integer> cameraIDs;
    private final ArrayList<Integer> targetIDs;

    private final ArrayList<Integer> fiducialCornerIDs;
    private final ArrayList<Pair<Double, Double>> measurements;
    private final ArrayList<Pair<Double, Double>> sigmas;

    public VisionMeasurement(final int id) {
        this.id = id;
        this.cameraIDs = new ArrayList<>();
        this.targetIDs = new ArrayList<>();
        this.fiducialCornerIDs = new ArrayList<>();
        this.measurements = new ArrayList<>();
        this.sigmas = new ArrayList<>();
    }

    public void setId(final int id) {
        this.id = id;
    }

    public void addMeasurement(final int cameraId, final int targetId, final int fiducialCornerId,
        final Pair<Double, Double> measurement, final Pair<Double, Double> sigma) {
        cameraIDs.add(cameraId);
        targetIDs.add(targetId);
        fiducialCornerIDs.add(fiducialCornerId);
        measurements.add(measurement);
        sigmas.add(sigma);
    }

    public double[] serialize() {
        final int dataLength = 7;
        final double[] data = new double[1 + dataLength * measurements.size()];
        data[0] = (double) this.id;
        for (int i = 0; i < measurements.size(); i++) {
            data[dataLength * i + 1] = (double) cameraIDs.get(i); // camera ID
            data[dataLength * i + 2] = (double) targetIDs.get(i); // target ID
            data[dataLength * i + 3] = (double) fiducialCornerIDs.get(i); // fiducial corner ID
            data[dataLength * i + 4] = measurements.get(i).getFirst(); // bearing
            data[dataLength * i + 5] = measurements.get(i).getSecond(); // elevation
            data[dataLength * i + 6] = sigmas.get(i).getFirst(); // bearing sigma
            data[dataLength * i + 7] = sigmas.get(i).getSecond(); // elevation sigma
        }
        return data;
    }

}
