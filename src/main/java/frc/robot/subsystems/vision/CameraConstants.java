package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConstants(String cameraName, String cameraIP, Transform3d robotToCamera,
    int resWidth, int resHeight, double fov) {
}
