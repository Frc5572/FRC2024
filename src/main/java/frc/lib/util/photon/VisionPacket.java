package frc.lib.util.photon;

import java.util.List;

public class VisionPacket {

    private final boolean hasTargets;
    private final Target bestTarget;
    private final List<Target> targets;
    private double captureTimestamp;

    public VisionPacket(boolean hasTargets, Target bestTarget, List<Target> targets,
        double captureTimestamp) {
        this.hasTargets = hasTargets;
        this.bestTarget = bestTarget;
        this.targets = targets;
        this.captureTimestamp = captureTimestamp;
    }

    public boolean hasTargets() {
        return hasTargets;
    }

    public Target getBestTarget() {
        return bestTarget;
    }

    public List<Target> getTargets() {
        return targets;
    }

    public double getCaptureTimestamp() {
        return captureTimestamp;
    }



}
