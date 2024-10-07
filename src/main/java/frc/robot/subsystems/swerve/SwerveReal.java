package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;

/** Real Class for Swerve */
public class SwerveReal implements SwerveIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    /** Real Swerve Initializer */
    public SwerveReal() {}

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();

    }

    public SwerveModule createSwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
        int cancoderID, Rotation2d angleOffset) {
        return new SwerveModule(moduleNumber, angleOffset,
            new SwerveModuleReal(driveMotorID, angleMotorID, cancoderID));
    }

    @Override
    public SwerveModule[] createModules() {
        return new SwerveModule[] {
            createSwerveModule(0, Constants.Swerve.Mod0.driveMotorID,
                Constants.Swerve.Mod0.angleMotorID, Constants.Swerve.Mod0.canCoderID,
                Constants.Swerve.Mod0.angleOffset),
            createSwerveModule(1, Constants.Swerve.Mod1.driveMotorID,
                Constants.Swerve.Mod1.angleMotorID, Constants.Swerve.Mod1.canCoderID,
                Constants.Swerve.Mod1.angleOffset),
            createSwerveModule(2, Constants.Swerve.Mod2.driveMotorID,
                Constants.Swerve.Mod2.angleMotorID, Constants.Swerve.Mod2.canCoderID,
                Constants.Swerve.Mod2.angleOffset),
            createSwerveModule(3, Constants.Swerve.Mod3.driveMotorID,
                Constants.Swerve.Mod3.angleMotorID, Constants.Swerve.Mod3.canCoderID,
                Constants.Swerve.Mod3.angleOffset)};
    }

}
