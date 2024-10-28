package frc.robot.subsystems.elevator_wrist;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.sim.SimulatedPumbaa;

/**
 * Elevator Wrist IO Sim
 */
public class ElevatorWristIOSim implements ElevatorWristIO {

    private final SimulatedPumbaa sim;

    /**
     * Elevator Wrist IO Sim
     */
    public ElevatorWristIOSim(SimulatedPumbaa sim) {
        this.sim = sim;
    }

    private static final double ELEVATOR_START_HEIGHT =
        frc.robot.Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT;
    private static final double ELEVATOR_MIN_HEIGHT =
        frc.robot.Constants.ElevatorWristConstants.ELEVATOR_REF_1_HEIGHT;
    private static final double ELEVATOR_MAX_HEIGHT =
        frc.robot.Constants.ElevatorWristConstants.ELEVATOR_REF_2_HEIGHT;
    private static final double ELEVATOR_TIME_RANGE = 3.0;

    private static final double ELEVATOR_MAX_SPEED =
        (ELEVATOR_MAX_HEIGHT - ELEVATOR_MIN_HEIGHT) / ELEVATOR_TIME_RANGE;

    public static final double ELEVATOR_M;
    public static final double ELEVATOR_B;

    static {
        //
        ELEVATOR_M =
            (frc.robot.Constants.ElevatorWristConstants.ELEVATOR_REF_2_ANGLE_MEASURED.getRotations()
                - frc.robot.Constants.ElevatorWristConstants.ELEVATOR_REF_1_ANGLE_MEASURED
                    .getRotations())
                / (frc.robot.Constants.ElevatorWristConstants.ELEVATOR_REF_2_HEIGHT
                    - frc.robot.Constants.ElevatorWristConstants.ELEVATOR_REF_1_HEIGHT);
        // meas_1 * m + b = act_1
        // b = act_1 - meas_1 * m
        ELEVATOR_B =
            frc.robot.Constants.ElevatorWristConstants.ELEVATOR_REF_1_ANGLE_MEASURED.getRotations()
                - frc.robot.Constants.ElevatorWristConstants.ELEVATOR_REF_1_HEIGHT * ELEVATOR_M;
    }

    private static final double WRIST_START_ANGLE = 0.0;
    private static final double WRIST_MAX_ANGLE = 55.0;
    private static final double WRIST_MIN_ANGLE = -30.0;
    private static final double WRIST_TIME_RANGE = 0.4;


    private static final double WRIST_MAX_SPEED =
        (WRIST_MAX_ANGLE - WRIST_MIN_ANGLE) / WRIST_TIME_RANGE;

    public static final double WRIST_M;
    public static final double WRIST_B;

    static {
        WRIST_M = (frc.robot.Constants.ElevatorWristConstants.WRIST_REF_2_ANGLE_MEASURED
            .getRotations()
            - frc.robot.Constants.ElevatorWristConstants.WRIST_REF_1_ANGLE_MEASURED.getRotations())
            / (frc.robot.Constants.ElevatorWristConstants.WRIST_REF_2_ANGLE_ACTUAL.getRotations()
                - frc.robot.Constants.ElevatorWristConstants.WRIST_REF_1_ANGLE_ACTUAL
                    .getRotations());
        // meas_1 * m + b = act_1
        // b = act_1 - meas_1 * m
        WRIST_B =
            frc.robot.Constants.ElevatorWristConstants.WRIST_REF_1_ANGLE_MEASURED.getRotations()
                - frc.robot.Constants.ElevatorWristConstants.WRIST_REF_1_ANGLE_ACTUAL.getRotations()
                    * WRIST_M;
    }

    private double elevatorCurrentSpeed = 0.0;
    private double elevatorCurrentPosition = ELEVATOR_START_HEIGHT;
    private double wristCurrentSpeed = 0.0;
    private double wristCurrentPosition = WRIST_START_ANGLE;

    @Override
    public void setElevatorVoltage(double voltage) {
        voltage = MathUtil.clamp(-voltage, -12.0, 12.0);
        elevatorCurrentSpeed = ELEVATOR_MAX_SPEED * (voltage / 12.0);
    }

    @Override
    public void setWristVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12.0, 12.0);
        wristCurrentSpeed = WRIST_MAX_SPEED * (voltage / 12.0);
    }

    @Override
    public void updateInputs(ElevatorWristInputs inputs) {
        elevatorCurrentPosition += elevatorCurrentSpeed * LoggedRobot.defaultPeriodSecs;
        elevatorCurrentPosition =
            MathUtil.clamp(elevatorCurrentPosition, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT);
        double elevatorMeas = ELEVATOR_M * elevatorCurrentPosition + ELEVATOR_B;
        inputs.leftElevatorRelativeEncRawValue = elevatorMeas;
        inputs.rightElevatorRelativeEncRawValue = elevatorMeas;

        wristCurrentPosition += wristCurrentSpeed * LoggedRobot.defaultPeriodSecs;
        wristCurrentPosition =
            MathUtil.clamp(wristCurrentPosition, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE);
        double wristMeas = WRIST_M * Units.degreesToRotations(wristCurrentPosition) + WRIST_B;

        inputs.wristAbsoluteEncRawValue = wristMeas;

        this.sim.setElevatorWrist(elevatorCurrentPosition,
            Rotation2d.fromDegrees(wristCurrentPosition));
    }

}
