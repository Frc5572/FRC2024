package frc.robot.subsystems.elevator_wrist;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Elevator and Wrist Subsystem
 */
public class ElevatorWrist implements Subsystem {
    public ElevatorWristIO io;
    public ElevatorWristInputsAutoLogged inputs = new ElevatorWristInputsAutoLogged();


    ProfiledPIDController elevatorPIDController =
        new ProfiledPIDController(Constants.ElevatorWristConstants.PID.ELEVATOR_KP,
            Constants.ElevatorWristConstants.PID.ELEVATOR_KI,
            Constants.ElevatorWristConstants.PID.ELEVATOR_KD,
            new TrapezoidProfile.Constraints(
                Constants.ElevatorWristConstants.PID.ELEVATOR_MAX_VELOCITY,
                Constants.ElevatorWristConstants.PID.ELEVATOR_MAX_ACCELERATION));

    ProfiledPIDController wristPIDController =
        new ProfiledPIDController(Constants.ElevatorWristConstants.PID.WRIST_KP,
            Constants.ElevatorWristConstants.PID.WRIST_KI,
            Constants.ElevatorWristConstants.PID.WRIST_KD,
            new TrapezoidProfile.Constraints(
                Constants.ElevatorWristConstants.PID.WRIST_MAX_VELOCITY,
                Constants.ElevatorWristConstants.PID.WRIST_MAX_ACCELERATION));


    private ElevatorFeedforward elevatorFeedForward =
        new ElevatorFeedforward(Constants.ElevatorWristConstants.PID.ELEVATOR_KS,
            Constants.ElevatorWristConstants.PID.ELEVATOR_KG,
            Constants.ElevatorWristConstants.PID.ELEVATOR_KV);

    private ArmFeedforward wristFeedForward =
        new ArmFeedforward(Constants.ElevatorWristConstants.PID.WRIST_KS,
            Constants.ElevatorWristConstants.PID.WRIST_KG,
            Constants.ElevatorWristConstants.PID.WRIST_KV);

    public ElevatorWrist(ElevatorWristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorWrist", inputs);

        double elevatorPIDValue = elevatorPIDController.calculate(elevatorDistanceTraveled());
        double wristPIDValue = wristPIDController.calculate(inputs.wristAbsoluteEncRawValue);

        double elevatorFeedForwardValue =
            elevatorFeedForward.calculate(0, 0, elevatorPIDController.getPeriod());

        double wristFeedForwardValue =
            wristFeedForward.calculate(0, 0, wristPIDController.getPeriod());

        io.setElevatorVoltage(elevatorFeedForwardValue + elevatorPIDValue);
        io.setWristVoltage(wristFeedForwardValue + wristPIDValue);

        Logger.recordOutput("/ElevatorWrist/Elevator/VoltageFromPID", elevatorPIDValue);
        Logger.recordOutput("/ElevatorWrist/Elevator/VoltageFromFeedForward",
            elevatorFeedForwardValue);
        Logger.recordOutput("/ElevatorWrist/Elevator/TotalVoltage",
            elevatorPIDValue + elevatorFeedForwardValue);

        Logger.recordOutput("/ElevatorWrist/Wrist/VoltageFromPID", wristPIDValue);
        Logger.recordOutput("/ElevatorWrist/Wrist/VoltageFromFeedForward", wristFeedForwardValue);
        Logger.recordOutput("/ElevatorWrist/Wrist/VoltageFromFeedForward",
            wristFeedForwardValue + wristPIDValue);

    }

    /**
     * Command to move Elevator and Wrist to set positions
     *
     * @param height The height of the elevator in meters
     * @param angle The angle of the wrist in {@link Rotation2d}
     *
     * @return A {@link Command}
     */
    public Command goToPosition(double height, Rotation2d angle) {
        return Commands.runOnce(() -> {
            elevatorPIDController.setGoal(height);
            wristPIDController.setGoal(angle.getRotations());
        }).andThen(Commands
            .waitUntil(() -> wristPIDController.atGoal() && elevatorPIDController.atGoal()));
    }

    /**
     * Command to continuously move the Elevator and Wrist to an ever changing position
     *
     * @param height A {@link DoubleSupplier} to provide the height of the elevator in meters
     * @param angle A {@link Supplier} of {@link Rotation2d} of angle of the wrist in
     *        {@link Rotation2d}
     *
     * @return A {@link Command}
     */
    public Command followPosition(DoubleSupplier height, Supplier<Rotation2d> angle) {
        return Commands.run(() -> {
            elevatorPIDController.setGoal(height.getAsDouble());
            wristPIDController.setGoal(angle.get().getRotations());
        });
    }

    /**
     * Get the height in meters of the elevator based on the rotations of the motor
     *
     * @return Height of elevator in meters
     */
    public double elevatorDistanceTraveled() {
        return inputs.elevatorRelativeEncRawValue * 0.111715034761762;
    }
}
