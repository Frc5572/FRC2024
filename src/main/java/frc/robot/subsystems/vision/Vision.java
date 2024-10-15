package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.photon.PhotonCameraWrapper;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private final PhotonCameraWrapper[] cameras;

    public Vision(boolean isSim) {
        super("Vision");
        cameras = new PhotonCameraWrapper[] {
            new PhotonCameraWrapper(Constants.CameraConstants.FrontRightFacingCamera.CAMERA_NAME,
                Constants.CameraConstants.FrontRightFacingCamera.CAMERA_IP,
                Constants.CameraConstants.FrontRightFacingCamera.KCAMERA_TO_ROBOT)};
    }

    @Override
    public void periodic() {
        for (var cam : cameras) {
            cam.periodic();
        }
        // TODO
        // https://github.com/Mechanical-Advantage/RobotCode2024/blob/a025615a52193b7709db7cf14c51c57be17826f2/src/main/java/org/littletonrobotics/frc2024/subsystems/apriltagvision/AprilTagVision.java#L59
    }

}
