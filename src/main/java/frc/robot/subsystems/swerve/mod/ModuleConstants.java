package frc.robot.subsystems.swerve.mod;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import lombok.Builder;

@Builder
public class ModuleConstants {

    public final double ffkS;
    public final double ffkV;
    public final double ffkT;
    public final double drivekP;
    public final double drivekD;
    public final double anglekP;
    public final double anglekD;
    public final double driveReduction;
    public final double angleReduction;

    public final DCMotor driveMotor;
    public final DCMotor angleMotor;
    public final Voltage driveFrictionVoltage;
    public final Voltage angleFrictionVoltage;
    public final double wheelCoeffFriction;
    public final MomentOfInertia angleMomentOfInertia;
    public final Distance wheelRadius;
    public final Current slipCurrent;

}
