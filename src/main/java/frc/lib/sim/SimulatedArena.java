package frc.lib.sim;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.FieldConstants;

public class SimulatedArena {

    private static class ShotNote {
        public Pose3d pose;
        public Translation3d velocity;

        public ShotNote(Pose3d pose, Translation3d velocity) {
            this.pose = pose;
            this.velocity = velocity;
        }

        public void update(double dt) {
            velocity = velocity.minus(new Translation3d(0, 0, 9.81 * dt));
            pose = pose.plus(new Transform3d(velocity.times(dt), NO_ROT));
        }
    }

    private static final double NOTE_HEIGHT = Units.inchesToMeters(1.7);
    private Set<SimulatedPumbaa> robots = new HashSet<>();
    private int id = 0;
    private static final Rotation3d NO_ROT = new Rotation3d();
    private List<Pose3d> notes = Stream
        .concat(Arrays.stream(FieldConstants.StagingLocations.centerlineTranslations),
            Arrays.stream(FieldConstants.StagingLocations.spikeTranslations))
        .map((pos) -> new Pose3d(new Translation3d(pos.getX(), pos.getY(), NOTE_HEIGHT), NO_ROT))
        .collect(Collectors.toList());
    public List<ShotNote> shotNotes = new ArrayList<>();

    public SimulatedPumbaa newPumbaa() {
        SimulatedPumbaa dt = new SimulatedPumbaa(this.id++);
        robots.add(dt);
        return dt;
    }

    public void update(double dt) {
        robots: for (SimulatedPumbaa robot : this.robots) {
            Logger.recordOutput("Viz/Robot" + robot.id, robot.getPose());
            robot.advanceNote(dt, this);
            if (robot.couldIntake()) {
                System.out.println("checking");
                for (int i = 0; i < notes.size(); i++) {
                    double distance = notes.get(i).toPose2d().getTranslation()
                        .minus(robot.getPose().getTranslation()).getNorm();
                    System.out.println(i + ": " + distance);
                    if (distance < 0.6) {
                        notes.remove(i);
                        robot.intakeOneNote();
                        continue robots;
                    }
                }
            }
        }
        for (int i = 0; i < shotNotes.size(); i++) {
            shotNotes.get(i).update(dt);
            if (shotNotes.get(i).pose.getZ() < NOTE_HEIGHT) {
                notes.add(new Pose3d(shotNotes.get(i).pose.getX(), shotNotes.get(i).pose.getY(),
                    NOTE_HEIGHT, NO_ROT));
                shotNotes.remove(i);
                i--;
                continue;
            }
        }
        Logger.recordOutput("Viz/Notes", this.notes.toArray(Pose3d[]::new));
        Logger.recordOutput("Viz/ShotNotes",
            this.shotNotes.stream().map((shotNote) -> shotNote.pose).toArray(Pose3d[]::new));
    }

    void shootNote(Pose3d pose, Translation3d velocity) {
        this.shotNotes.add(new ShotNote(pose, velocity));
    }

}
