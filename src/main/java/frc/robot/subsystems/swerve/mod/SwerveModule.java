package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.Logger;

public final class SwerveModule {

    private final ModuleConfig config;
    private final SwerveModuleAngleIO angleIO;
    private final SwerveModuleDriveIO driveIO;

    private final String angleKey;
    private final String driveKey;

    private final SwerveModuleAngleInputsAutoLogged angleInputs =
        new SwerveModuleAngleInputsAutoLogged();

    private final SwerveModuleDriveInputsAutoLogged driveInputs =
        new SwerveModuleDriveInputsAutoLogged();

    public SwerveModule(ModuleConfig config, SwerveModuleAngleIO angleIO,
        SwerveModuleDriveIO driveIO) {
        this.config = config;
        this.angleIO = angleIO;
        this.driveIO = driveIO;
        this.angleKey = "Swerve/Mod[" + config.moduleNumber + "]/angle";
        this.driveKey = "Swerve/Mod[" + config.moduleNumber + "]/drive";
    }

    public void updateInputs() {
        angleIO.updateAngleInputs(angleInputs);
        driveIO.updateDriveInputs(driveInputs);
        Logger.processInputs(angleKey, angleInputs);
        Logger.processInputs(driveKey, driveInputs);
    }

    public void periodic() {

    }

}
