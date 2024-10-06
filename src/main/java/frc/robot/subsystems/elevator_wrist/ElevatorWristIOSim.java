package frc.robot.subsystems.elevator_wrist;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ElevatorWristIOSim implements ElevatorWristIO {

    private final DCMotor wrist = DCMotor.getNEO(1);
    private final Encoder m_encoder = new Encoder(0, 1);
    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(wrist, 60.0,
        SingleJointedArmSim.estimateMOI(Units.inchesToMeters(12), 8.0), Units.inchesToMeters(12),
        Constants.ElevatorWristConstants.SetPoints.MIN_ANGLE.getRadians(),
        Constants.ElevatorWristConstants.SetPoints.MAX_ANGLE.getRadians(), true, 0.0);
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

    public ElevatorWristIOSim() {

    }

    @Override
    public void updateInputs(ElevatorWristInputs inputs) {
        m_armSim.update(LoggedRobot.defaultPeriodSecs);
        m_encoderSim.setDistance(m_armSim.getAngleRads());
        RobotContainer.m_wrist.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
        inputs.wristAbsoluteEncRawValue = Units.radiansToRotations(m_encoderSim.getDistance());
    }

    @Override
    public void setWristVoltage(double v) {
        m_armSim.setInputVoltage(v);
    }

}
