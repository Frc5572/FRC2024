package frc.lib.sim;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.FieldConstants;

public class SimulatedArena {

    private Set<SimulatedPumbaa> robots = new HashSet<>();
    private int id = 0;
    private static final Rotation3d NO_ROT = new Rotation3d();
    private List<Pose3d> notes = Stream
        .concat(Arrays.stream(FieldConstants.StagingLocations.centerlineTranslations),
            Arrays.stream(FieldConstants.StagingLocations.spikeTranslations))
        .map((pos) -> new Pose3d(new Translation3d(pos.getX(), pos.getY(), Units.inchesToMeters(2)),
            NO_ROT))
        .collect(Collectors.toList());

    public SimulatedPumbaa newPumbaa() {
        SimulatedPumbaa dt = new SimulatedPumbaa(this.id++);
        robots.add(dt);
        return dt;
    }

    public void update(double dt) {
        robots: for (SimulatedPumbaa robot : this.robots) {
            robot.advanceNote(dt, this);
            if (robot.couldIntake()) {
                for (int i = 0; i < notes.size(); i++) {
                    if (notes.get(i).toPose2d().minus(robot.getPose()).getTranslation()
                        .getNorm() < 0.5) {
                        notes.remove(i);
                        robot.intakeOneNote();
                        continue robots;
                    }
                }
            }
        }
        Logger.recordOutput("Viz/Notes", this.notes.toArray(Pose3d[]::new));
    }

    void shootNote(Pose3d pose) {

    }

}