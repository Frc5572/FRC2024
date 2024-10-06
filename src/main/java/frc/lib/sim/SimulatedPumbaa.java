package frc.lib.sim;

import edu.wpi.first.math.geometry.Pose2d;

public class SimulatedPumbaa {

    public final int id;

    SimulatedPumbaa(int id) {
        this.id = id;
    }

    private Pose2d pose = new Pose2d();
    private double intake = 0.0;
    private double indexer = 0.0;
    private double notePosition = 0.0;
    private boolean hasNote = false;

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public boolean couldIntake() {
        return intake > 0.2;
    }

    public void setIntake(double value) {
        this.intake = value;
    }

    public void setIndexer(double value) {
        this.indexer = value;
    }

    public void intakeOneNote() {
        this.notePosition = 0.0;
        this.hasNote = true;
    }

    public void advanceNote(double dt, SimulatedArena arena) {
        if (hasNote) {
            if (notePosition < 0.85) {
                notePosition += intake * dt;
            } else if (indexer < -0.2) {
                this.hasNote = false;
                // Spit out back
            } else if (notePosition > 1.0) {
                this.hasNote = false;
                // Spit out front
            } else {
                notePosition += indexer * dt;
            }
        }
    }

    public boolean lowerBeamBreak() {
        return hasNote && notePosition < 0.8;
    }

    public boolean upperBeamBreak() {
        return hasNote && notePosition > 0.9 && notePosition < 1.0;
    }

}
