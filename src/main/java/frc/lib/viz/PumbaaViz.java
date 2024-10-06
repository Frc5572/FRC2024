package frc.lib.viz;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class PumbaaViz {

    private final String prefix;
    private Pose3d elevatorBottomPose;
    private Pose3d elevatorTopPose;
    private Pose3d shooterPose;
    private Pose3d notePose;
    private NoteLocation noteLocation = NoteLocation.None;

    private final Rotation3d NO_ROT = new Rotation3d();
    private final Pose3d INTAKE_POSE = new Pose3d(-0.342037, 0.0, 0.205429,
        new Rotation3d(0.0, Units.degreesToRadians(-73.2153), 0.0));

    private final double shooterHeight = 0.538339;
    private final double indexerBack = 0.150388;
    private final double shooterFront = 0.26127;

    public PumbaaViz(String prefix) {
        this.prefix = prefix;
        this.elevatorBottomPose = new Pose3d();
        this.elevatorTopPose = new Pose3d();
    }

    public void setElevatorWrist(double height, Rotation2d wristAngle) {
        height =
            Units.inchesToMeters(height - Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT);
        shooterPose = new Pose3d(new Translation3d(0.0, 0.0, shooterHeight + height),
            new Rotation3d(0.0, -wristAngle.getRadians(), 0.0));
        elevatorBottomPose = new Pose3d(0, 0, height / 2.0, NO_ROT);
        elevatorTopPose = new Pose3d(0, 0, height, NO_ROT);
        if (noteLocation == NoteLocation.Shooter) {
            double indexerBackX = indexerBack * wristAngle.getCos();
            double indexerBackZ = indexerBack * wristAngle.getSin();
            notePose = shooterPose.plus(new Transform3d(-indexerBackX, 0, 0, NO_ROT));
        }
    }

    public void setPose(Pose2d pose) {

    }

    public static enum NoteLocation {
        Intake, Shooter, None
    }

    public void setNoteLocation(NoteLocation location) {
        switch (location) {
            case Intake:
                this.notePose = INTAKE_POSE;
                break;
            case Shooter:
                break;
            default:
                this.notePose = new Pose3d(0, 0, -100.0, NO_ROT);
                break;
        }
        this.noteLocation = location;
    }

    public void update() {
        Logger.recordOutput(prefix + "/components",
            new Pose3d[] {shooterPose, elevatorBottomPose, elevatorTopPose, notePose});
    }

}
