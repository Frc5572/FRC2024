package frc.lib.sim;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class SimulatedPumbaa {

    public final int id;

    SimulatedPumbaa(int id) {
        this.id = id;
    }

    private static final double SHOOTER_DIV = 100.0;
    private static final double SHOOTER_FRONT = 0.271162;
    private Pose2d pose = new Pose2d();
    private double intake = 0.0;
    private double indexer = 0.0;
    private double notePosition = 0.9;
    private boolean hasNote = true;
    private double shooterSpeed = 0.0;
    private double height = 24.0;
    private Rotation2d wristAngle = new Rotation2d();

    public double notePosition() {
        return this.notePosition;
    }

    public void setShooterSpeed(double speed) {
        this.shooterSpeed = speed;
    }

    public void setElevatorWrist(double height, Rotation2d wristAngle) {
        this.height = height;
        this.wristAngle = wristAngle;
    }

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
        Rotation2d yaw = this.getPose().getRotation();
        Rotation2d pitch = this.wristAngle;
        double x = yaw.getCos() * pitch.getCos();
        double y = yaw.getSin() * pitch.getCos();
        Translation3d shooterSpeed = new Translation3d(x, y, pitch.getSin());
        shooterSpeed = shooterSpeed.times(this.shooterSpeed / SHOOTER_DIV);
        Logger.recordOutput("Viz/shooterSpeed", shooterSpeed);
        Logger.recordOutput("Viz/shooterDir", yaw);
        if (hasNote) {
            if (notePosition < 0.85) {
                notePosition += intake * dt;
            } else if (indexer < -0.2) {
                this.hasNote = false;
                arena.shootNote(new Pose3d(this.getPose()), new Translation3d());
            } else if (notePosition > 1.0) {
                this.hasNote = false;
                Translation3d t = new Translation3d(
                    this.getPose().getX() + SHOOTER_FRONT * this.getPose().getRotation().getCos(),
                    this.getPose().getY() + SHOOTER_FRONT * this.getPose().getRotation().getSin(),
                    Units.inchesToMeters(height));
                Rotation3d r = new Rotation3d(0.0, wristAngle.getRadians(),
                    this.getPose().getRotation().getRadians());
                Pose3d fromPose = new Pose3d(t, r);
                Logger.recordOutput("Viz/fromPose", fromPose);
                arena.shootNote(fromPose, shooterSpeed);
            } else {
                notePosition += indexer * dt;
            }
        }
    }

    public boolean hasNote() {
        return this.hasNote;
    }

    public boolean lowerBeamBreak() {
        return hasNote && notePosition < 0.8;
    }

    public boolean upperBeamBreak() {
        return hasNote && notePosition > 0.9 && notePosition < 1.0;
    }

}
