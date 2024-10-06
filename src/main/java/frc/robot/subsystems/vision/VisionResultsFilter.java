package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionResultsFilter {
    String getFilterImplementationName();

    boolean isResultValid(Pose3d robotPoseEstimation);
}
