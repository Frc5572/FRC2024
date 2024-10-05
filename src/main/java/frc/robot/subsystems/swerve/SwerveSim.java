package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleSim;

/** Real Class for Swerve */
public class SwerveSim implements SwerveIO {

    /** Real Swerve Initializer */
    public SwerveSim() {}

    @Override
    public void updateInputs(SwerveInputs inputs) {
        // inputs.yaw = 0;
        // inputs.pitch = 0;
        // inputs.roll = 0;

    }

    @Override
    public SwerveModule createSwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, driveMotorID, angleMotorID, cancoderID, angleOffset,
            new SwerveModuleSim(moduleNumber));
    }

}
