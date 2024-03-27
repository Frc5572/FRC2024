package frc.lib.util.rospf;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseEstimate {

    private final int id;
    private final Pose2d pose;
    private final boolean hasVision;

    public PoseEstimate(int id, Pose2d pose, boolean hasVision) {
        this.id = id;
        this.pose = pose;
        this.hasVision = hasVision;
    }

    public int getID() {
        return id;
    }

    public Pose2d getPose() {
        return pose;
    }

    public boolean hasVision() {
        return hasVision;
    }

}
