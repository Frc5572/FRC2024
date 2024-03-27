package frc.lib.util.rospf;

import edu.wpi.first.math.geometry.Pose2d;

public class OdometryMeasurement {

    private int id;
    private final Pose2d pose;
    private final Pose2d sigmas;

    public OdometryMeasurement(final int id, final Pose2d pose, final Pose2d sigmas) {
        this.id = id;
        this.pose = pose;
        this.sigmas = sigmas;
    }

    public void setId(final int id) {
        this.id = id;
    }

    public double[] serialize() {
        return new double[] {(double) this.id, this.pose.getX(), this.pose.getY(),
            this.pose.getRotation().getRadians(), this.sigmas.getX(), this.sigmas.getY(),
            this.sigmas.getRotation().getRadians(),};
    }

}
