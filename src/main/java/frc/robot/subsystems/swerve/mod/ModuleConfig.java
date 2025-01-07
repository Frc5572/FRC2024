package frc.robot.subsystems.swerve.mod;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Builder;

@Builder
public class ModuleConfig {

    public final int moduleNumber;
    public final int driveId;
    public final int angleId;
    public final int absoluteEncoderId;
    public final Rotation2d absoluteEncoderOffset;
    public final boolean angleMotorInverted;

}
