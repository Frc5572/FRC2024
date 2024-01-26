package frc.robot.subsystems.elevatorWrist;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Elevator and Wrist Subsystem
 */
public class ElevatorWrist implements Subsystem {
    public ElevatorWristIO io;
    public ElevatorWristInputsAutoLogged inputs = new ElevatorWristInputsAutoLogged();

    TrapezoidProfile.Constraints elevatorProfileConstraints =
        new TrapezoidProfile.Constraints(Constants.ElevatorWristConstants.ELEVATOR_MAX_VELOCITY,
            Constants.ElevatorWristConstants.ELEVATOR_MAX_ACCELERATION);
    TrapezoidProfile elevatorProfile = new TrapezoidProfile(elevatorProfileConstraints);

    ProfiledPIDController elevatorPIDController = new ProfiledPIDController(
        Constants.ElevatorWristConstants.ELEVATOR_KP, Constants.ElevatorWristConstants.ELEVATOR_KI,
        Constants.ElevatorWristConstants.ELEVATOR_KD, elevatorProfileConstraints);

    ProfiledPIDController wristPIDController =
        new ProfiledPIDController(Constants.ElevatorWristConstants.WRIST_KP,
            Constants.ElevatorWristConstants.WRIST_KI, Constants.ElevatorWristConstants.WRIST_KD,
            new TrapezoidProfile.Constraints(Constants.ElevatorWristConstants.WRIST_MAX_VELOCITY,
                Constants.ElevatorWristConstants.WRIST_MAX_ACCELERATION));

    private ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
        Constants.ElevatorWristConstants.ELEVATOR_KS, Constants.ElevatorWristConstants.ELEVATOR_KG,
        Constants.ElevatorWristConstants.ELEVATOR_KV);

    private ArmFeedforward wristFeedForward =
        new ArmFeedforward(Constants.ElevatorWristConstants.WRIST_KS,
            Constants.ElevatorWristConstants.WRIST_KG, Constants.ElevatorWristConstants.WRIST_KV);

    public ElevatorWrist(ElevatorWristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("ElevatorWrist", inputs);


        State next = elevatorProfile.calculate(wristPIDController.getPeriod(),
            wristPIDController.getSetpoint(), wristPIDController.getGoal());

        double elevatorPIDValue =
            elevatorPIDController.calculate(inputs.elevatorRelativeEncRawValue);
        double wristPIDValue = wristPIDController.calculate(inputs.wristAbsoluteEncRawValue);

        double elevatorFeedForwardValue =
            elevatorFeedForward.calculate(wristPIDController.getSetpoint().velocity, next.velocity,
                wristPIDController.getPeriod());
    }

}
