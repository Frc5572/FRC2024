package frc.lib.util.rospf;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveDriveOdometryMeasurement {

    private final Rotation2d gyroAngle;
    private final SwerveModulePosition[] modulePositions;

    public SwerveDriveOdometryMeasurement(final Rotation2d gyroAngle,
        final SwerveModulePosition[] modulePositions) {
        this.gyroAngle = gyroAngle;
        this.modulePositions = modulePositions;
    }

    public Rotation2d getGyroAngle() {
        return gyroAngle;
    }

    public SwerveModulePosition[] getModulePositionStates() {
        return modulePositions;
    }
}
