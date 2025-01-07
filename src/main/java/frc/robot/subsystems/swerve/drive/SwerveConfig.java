package frc.robot.subsystems.swerve.drive;

import frc.robot.subsystems.swerve.mod.ModuleConfig;

public class SwerveConfig {

    public ModuleConfig frontLeft;
    public ModuleConfig frontRight;
    public ModuleConfig backLeft;
    public ModuleConfig backRight;

    public ModuleConfig[] modules() {
        return new ModuleConfig[] {frontLeft, frontRight, backLeft, backRight};
    }

}
