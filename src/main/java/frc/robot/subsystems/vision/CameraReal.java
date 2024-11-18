package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class CameraReal extends PhotonCamera implements CameraIO {

    public CameraReal(CameraConstants constants) {
        this(constants, true);
    }

    protected CameraReal(CameraConstants constants, boolean shouldUpload) {
        super(constants.cameraName());
    }

    @Override
    public void updateInputs(CameraInputs inputs) {
        inputs.result = super.getLatestResult();
        inputs.cameraMatrix = super.getCameraMatrix();
        inputs.distCoeffs = super.getDistCoeffs();
    }

}
