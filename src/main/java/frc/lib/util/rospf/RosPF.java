package frc.lib.util.rospf;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.function.Consumer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.photon.VisionPacket;
import frc.robot.OperatorState;

public class RosPF {

    private final PFTables networkTable = new PFTables(this::newEstimateListener);

    // ever increasing id. Used as a uuid of time that is guaranteed to be the same from measurments
    // to estimates.
    private int currentID = 0;

    // id -> time
    private final HashMap<Integer, Double> idToTimeMap = new HashMap<>();
    // time -> swerve odometry
    private final ConcurrentSkipListMap<Double, SwerveDriveOdometryMeasurement> timeToOdometryMap =
        new ConcurrentSkipListMap<>();

    // How far back we keep a history to associate vision measurements with pose estimates.
    private final double BUFFER_HISTORY = 10.0; // s

    // Onboard estimated pose history.
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(BUFFER_HISTORY);

    // time -> (odometry, vision)
    private final TreeMap<Double, Pair<OdometryMeasurement, VisionMeasurement>> timeToMeasurementMap =
        new TreeMap<>();

    // Continuous odometry from the last reset. Used to get odometry deltas.
    private final SwerveDriveOdometry rawOdometry;

    // Odometry played back whenever new estimate is recieved (to account for estimation latency).
    private final SwerveDriveOdometry playbackOdometry;

    // Latest raw pose from rospf.
    private PoseEstimate latestRawEstimate = new PoseEstimate(0, new Pose2d(), false);

    // Measurements within this amount of the current time are not considered final. This gives us a
    // change to associate a new vision measurement with odometry measurements despite being
    // asequent.
    private final double MUTABLE_TIME_BUFFER = 0.2;

    public RosPF(final SwerveDriveKinematics kinematics, final Rotation2d initialGyroAngle,
        final SwerveModulePosition[] modulePositions, final Pose2d priori,
        AprilTagFieldLayout targets) {
        rawOdometry =
            new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
        playbackOdometry =
            new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
        networkTable.publishTargets(targets);
    }

    public void resetPose(final Rotation2d gyroAngle, final SwerveModulePosition[] modulePositions,
        final Pose2d pose) {
        rawOdometry.resetPosition(gyroAngle, modulePositions, pose);
        playbackOdometry.resetPosition(gyroAngle, modulePositions, pose);
    }

    public Pose2d getOdometryPose() {
        return rawOdometry.getPoseMeters();
    }

    public Pose2d getPose() {
        return playbackOdometry.getPoseMeters();
    }

    public Pose2d getRawPose() {
        return latestRawEstimate.getPose();
    }

    public void update(final SwerveDriveOdometryMeasurement odometry,
        final ArrayList<VisionPacket> visionPackets) {
        final double currentTime = Timer.getFPGATimestamp();
        final var curPose = rawOdometry.getPoseMeters();

        timeToOdometryMap.put(currentTime, odometry);
        poseBuffer.addSample(currentTime, curPose);
        rawOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
        playbackOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());

        timeToMeasurementMap.put(currentTime, new Pair<>(
            new OdometryMeasurement(0, curPose, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1))),
            null));

        for (int cameraID = 0; cameraID < visionPackets.size(); cameraID++) {
            final var vision = visionPackets.get(cameraID);
            if (!vision.hasTargets()) {
                continue;
            }

            final double measurmentTime = vision.getCaptureTimestamp();
            final var existingEntry = timeToMeasurementMap.get(measurmentTime);
            final var existingVisionMeasurement =
                existingEntry == null ? null : existingEntry.getSecond();
            final var visionMeasurement =
                existingVisionMeasurement == null ? new VisionMeasurement(0)
                    : existingVisionMeasurement;
            for (final var target : vision.getTargets()) {
                if (!OperatorState.tagFilter(target.getID())) {
                    continue;
                }
                if (target.getID() != -1) {
                    for (int cornerID = 0; cornerID < target.getCorners().size(); cornerID++) {
                        visionMeasurement.addMeasurement(cameraID, target.getID(), cornerID,
                            target.getCornerSphericalBE(cornerID), new Pair<>(0.1, 0.1));
                    }
                }
            }

            final Pose2d interpolatedPose = poseBuffer.getSample(measurmentTime).get();
            timeToMeasurementMap.put(measurmentTime,
                new Pair<>(new OdometryMeasurement(0, interpolatedPose,
                    new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.1))), visionMeasurement));
        }

        publishImmutableEntries();
    }

    private void publishImmutableEntries() {
        final double currentTime = Timer.getFPGATimestamp();

        final ArrayList<Double> times = new ArrayList<>(timeToMeasurementMap.keySet());
        for (final double time : times) {
            if (currentTime - time > MUTABLE_TIME_BUFFER) {
                idToTimeMap.put(currentID, time);

                // Always publish odometry
                final var measurement = timeToMeasurementMap.get(time);
                final var odometryMeasurement = measurement.getFirst();
                odometryMeasurement.setId(currentID);
                networkTable.publishOdometry(odometryMeasurement);

                // Publish vision if available
                final var visionMeasurement = measurement.getSecond();
                if (visionMeasurement != null) {
                    visionMeasurement.setId(currentID);
                    networkTable.publishVision(visionMeasurement);
                }

                timeToMeasurementMap.remove(time);
                currentID += 1;
            }
        }
    }

    private void newEstimateListener(final PoseEstimate estimate) {
        latestRawEstimate = estimate;
        if (!estimate.hasVision()) {
            return;
        }

        final double estimateTime = idToTimeMap.get(estimate.getID());
        Double curTime = timeToOdometryMap.ceilingKey(estimateTime);
        final var measurement = timeToOdometryMap.get(curTime);
        playbackOdometry.resetPosition(measurement.getGyroAngle(),
            measurement.getModulePositionStates(), estimate.getPose());

        // play back odometry
        while (curTime != null) {
            final SwerveDriveOdometryMeasurement lastMeasurement = timeToOdometryMap.get(curTime);
            playbackOdometry.update(lastMeasurement.getGyroAngle(),
                lastMeasurement.getModulePositionStates());
            curTime = timeToOdometryMap.higherKey(curTime);
        }
    }

    private static class PFTables {
        private final NetworkTable pfTable;
        private final DoubleArrayPublisher odometryPublisher;
        private final DoubleArrayPublisher visionPublisher;
        private final DoubleArrayPublisher targetPublisher;

        public PFTables(final Consumer<PoseEstimate> newEstimateListener) {
            final NetworkTableInstance inst = NetworkTableInstance.getDefault();
            pfTable = inst.getTable("rospf");
            odometryPublisher =
                pfTable.getDoubleArrayTopic("odometry").publish(PubSubOption.sendAll(true));
            visionPublisher =
                pfTable.getDoubleArrayTopic("vision").publish(PubSubOption.sendAll(true));
            targetPublisher =
                pfTable.getDoubleArrayTopic("targets").publish(PubSubOption.sendAll(true));

            final var estimateSubscriber = pfTable.getDoubleArrayTopic("pose")
                .subscribe(new double[] {}, PubSubOption.sendAll(true));
            inst.addListener(estimateSubscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    newEstimateListener
                        .accept(new PoseEstimate((int) event.valueData.value.getDoubleArray()[0],
                            new Pose2d(event.valueData.value.getDoubleArray()[1],
                                event.valueData.value.getDoubleArray()[2],
                                new Rotation2d(event.valueData.value.getDoubleArray()[3])),
                            event.valueData.value.getDoubleArray()[4] == 1.0 ? true : false));
                });
        }

        public void publishOdometry(final OdometryMeasurement odometry) {
            odometryPublisher.set(odometry.serialize());
        }

        public void publishVision(final VisionMeasurement vision) {
            visionPublisher.set(vision.serialize());
        }

        public void publishTargets(final AprilTagFieldLayout targets) {
            var tags = targets.getTags();
            final int dataLength = 7;
            final double[] data = new double[dataLength * tags.size()];
            for (int i = 0; i < tags.size(); i++) {
                data[dataLength * i] = tags.get(i).ID;
                data[dataLength * i + 1] = tags.get(i).pose.getX();
                data[dataLength * i + 2] = tags.get(i).pose.getY();
                data[dataLength * i + 3] = tags.get(i).pose.getZ();
                data[dataLength * i + 4] = tags.get(i).pose.getRotation().getX();
                data[dataLength * i + 5] = tags.get(i).pose.getRotation().getY();
                data[dataLength * i + 6] = tags.get(i).pose.getRotation().getZ();
            }
            targetPublisher.set(data);
        }

    }

}
