package frc.lib.viz;

import java.util.ArrayList;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.sim.SimulatedPumbaa;
import frc.robot.Constants;

public class PumbaaViz {

    private final String prefix;
    private Pose3d elevatorBottomPose;
    private Pose3d elevatorTopPose;
    private Pose3d shooterPose;
    private Pose3d notePose;
    private NoteLocation noteLocation = NoteLocation.None;
    private final SimulatedPumbaa sim;
    private Rotation2d wristAngle;
    private double height;
    private Pose2d robotPose = new Pose2d();

    private final Rotation3d NO_ROT = new Rotation3d();
    private final Pose3d NO_NOTE_POSE = new Pose3d(0, 0, -100.0, NO_ROT);
    private final Rotation3d INTAKE_ROTATION =
        new Rotation3d(0.0, Units.degreesToRadians(-73.2153), 0.0);
    private final Pose3d INTAKE_POSE = new Pose3d(-0.342037, 0.0, 0.205429,
        new Rotation3d(0.0, Units.degreesToRadians(-73.2153), 0.0));
    private final Translation3d INTAKE_TOP_POSE = new Translation3d(-0.295609, 0.0, 0.376448);
    private final Translation3d INTAKE_BOTTOM_POSE = new Translation3d(-0.380825, 0.0, 0.071435);
    private final Translation3d INTAKE_DIR = INTAKE_TOP_POSE.minus(INTAKE_BOTTOM_POSE);

    private final double shooterHeight = 0.538339;
    private final double indexerBack = 0.150388;

    public PumbaaViz(String prefix, SimulatedPumbaa sim) {
        this.prefix = prefix;
        this.sim = sim;
        this.elevatorBottomPose = new Pose3d();
        this.elevatorTopPose = new Pose3d();
    }

    public void setElevatorWrist(double height, Rotation2d wristAngle) {
        this.height = height;
        height =
            Units.inchesToMeters(height - Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT);
        this.wristAngle = wristAngle;
        shooterPose = new Pose3d(new Translation3d(0.0, 0.0, shooterHeight + height),
            new Rotation3d(0.0, -wristAngle.getRadians(), 0.0));
        elevatorBottomPose = new Pose3d(0, 0, height / 2.0, NO_ROT);
        elevatorTopPose = new Pose3d(0, 0, height, NO_ROT);
        if (noteLocation == NoteLocation.Shooter) {
            double indexerBackX = indexerBack * wristAngle.getCos();
            notePose = shooterPose.plus(new Transform3d(-indexerBackX, 0, 0, NO_ROT));
        }
    }

    public void setPose(Pose2d pose) {
        this.robotPose = pose;
    }

    public static enum NoteLocation {
        Intake, Shooter, None
    }

    public void setNoteLocation(NoteLocation location) {
        if (sim != null)
            return;
        switch (location) {
            case Intake:
                this.notePose = INTAKE_POSE;
                break;
            case Shooter:
                break;
            default:
                this.notePose = NO_NOTE_POSE;
                break;
        }
        this.noteLocation = location;
    }

    private static final double SHOOTER_FRONT = 0.271162;

    public Pose3d getShootFrom() {
        Translation3d t = new Translation3d(
            this.robotPose.getX() + SHOOTER_FRONT * this.robotPose.getRotation().getCos(),
            this.robotPose.getY() + SHOOTER_FRONT * this.robotPose.getRotation().getSin(),
            Units.inchesToMeters(height));
        Rotation3d r =
            new Rotation3d(0.0, wristAngle.getRadians(), this.robotPose.getRotation().getRadians());
        return new Pose3d(t, r);
    }

    ArrayList<Pose3d> trajectory = new ArrayList<>();

    public void update() {
        if (this.sim != null) {
            if (this.sim.hasNote()) {
                double notePosition = this.sim.notePosition();
                if (notePosition < 0.8) {
                    double notePositionPercent = notePosition / 0.85;
                    notePose =
                        new Pose3d(INTAKE_BOTTOM_POSE.plus(INTAKE_DIR.times(notePositionPercent)),
                            INTAKE_ROTATION);
                } else if (notePosition < 0.9) {
                    double notePositionPercent = (notePosition - 0.8) / 0.1;
                    double indexerBackX = indexerBack * wristAngle.getCos();
                    Pose3d topPose = shooterPose.plus(new Transform3d(-indexerBackX, 0, 0, NO_ROT));
                    Pose3d bottomPose = new Pose3d(INTAKE_TOP_POSE, INTAKE_ROTATION);
                    notePose = bottomPose.interpolate(topPose, notePositionPercent);
                } else {
                    double notePositionPercent = (notePosition - 0.9) / 0.1;
                    double indexerBackX =
                        indexerBack * -(notePositionPercent * 2.0 - 1.0) * wristAngle.getCos();
                    notePose = shooterPose.plus(new Transform3d(-indexerBackX, 0, 0, NO_ROT));
                }
            } else {
                this.notePose = NO_NOTE_POSE;
            }
        }
        recalculateTrajectory();
        Logger.recordOutput(prefix + "/trajectory", trajectory.toArray(Pose3d[]::new));
        Logger.recordOutput(prefix + "/components",
            new Pose3d[] {shooterPose, elevatorBottomPose, elevatorTopPose, notePose});
    }

    private static final double SHOOT_SPEED = 33.0;

    void recalculateTrajectory() {
        trajectory.clear();
        Pose3d startPose = getShootFrom();
        double vz = wristAngle.getSin() * SHOOT_SPEED;
        double vx = robotPose.getRotation().getCos() * SHOOT_SPEED;
        double vy = robotPose.getRotation().getSin() * SHOOT_SPEED;
        double prevZ = startPose.getZ();
        for (double t = 0.0; prevZ > 0.0; t += LoggedRobot.defaultPeriodSecs) {
            double x = startPose.getX() + vx * t;
            double y = startPose.getY() + vy * t;
            double z = startPose.getZ() + vz * t - 9.81 * t * t;
            trajectory.add(new Pose3d(x, y, z, new Rotation3d()));
            prevZ = z;
        }
    }

}
