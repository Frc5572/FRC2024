package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class CameraSimPhoton extends CameraReal {

    private static VisionSystemSim sim;
    private final PhotonCameraSim cameraSim;

    public CameraSimPhoton(CameraConstants constants) {
        super(constants, false);
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(constants.resWidth(), constants.resHeight(),
            Rotation2d.fromDegrees(constants.fov()));
        cameraProp.setCalibError(0.025, 0.08);
        cameraProp.setFPS(18);
        cameraProp.setAvgLatencyMs(60);
        cameraProp.setLatencyStdDevMs(30);
        if (sim == null) {
            sim = new VisionSystemSim("main");
        }
        cameraSim = new PhotonCameraSim(this, cameraProp);
        sim.addCamera(cameraSim, constants.robotToCamera());
    }

    public static void updateSimPosition(Pose3d pose) {
        sim.update(pose);
    }

    public static void updateSimPosition(Pose2d pose) {
        sim.update(pose);
    }

}
