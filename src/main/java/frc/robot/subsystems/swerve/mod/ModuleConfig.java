package frc.robot.subsystems.swerve.mod;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Builder;

/** Constants for an individual module. */
@Builder
public class ModuleConfig {

    /** Which module is this? */
    public final int moduleNumber;

    /** ID (usually CAN ID) for the drive motor */
    public final int driveId;

    /** ID (usually CAN ID) for the angle motor */
    public final int angleId;

    /** ID (usually CAN ID) for the absolute encoder */
    public final int absoluteEncoderId;

    /** Reading from the absolute encoder when facing "forward" */
    public final Rotation2d absoluteEncoderOffset;

    /** True if motor and encoder have opposite positive directions. */
    public final boolean angleMotorInverted;

}
