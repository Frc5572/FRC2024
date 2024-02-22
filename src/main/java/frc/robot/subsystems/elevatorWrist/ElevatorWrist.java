package frc.robot.subsystems.elevatorWrist;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Elevator and Wrist Subsystem
 */
public class ElevatorWrist extends SubsystemBase {
    public ElevatorWristIO io;
    public ElevatorWristInputsAutoLogged inputs = new ElevatorWristInputsAutoLogged();


    ProfiledPIDController elevatorPIDController =
        new ProfiledPIDController(Constants.ElevatorWristConstants.PID.ELEVATOR_KP,
            Constants.ElevatorWristConstants.PID.ELEVATOR_KI,
            Constants.ElevatorWristConstants.PID.ELEVATOR_KD,
            new TrapezoidProfile.Constraints(
                Constants.ElevatorWristConstants.PID.ELEVATOR_MAX_VELOCITY,
                Constants.ElevatorWristConstants.PID.ELEVATOR_MAX_ACCELERATION));

    PIDController wristPIDController =
        new PIDController(Constants.ElevatorWristConstants.PID.WRIST_KP,
            Constants.ElevatorWristConstants.PID.WRIST_KI,
            Constants.ElevatorWristConstants.PID.WRIST_KD);


    // private ElevatorFeedforward elevatorFeedForward =
    // new ElevatorFeedforward(Constants.ElevatorWristConstants.PID.ELEVATOR_KS,
    // Constants.ElevatorWristConstants.PID.ELEVATOR_KG,
    // Constants.ElevatorWristConstants.PID.ELEVATOR_KV);

    // private ArmFeedforward wristFeedForward =
    // new ArmFeedforward(Constants.ElevatorWristConstants.PID.WRIST_KS,
    // Constants.ElevatorWristConstants.PID.WRIST_KG,
    // Constants.ElevatorWristConstants.PID.WRIST_KV);

    public ElevatorWrist(ElevatorWristIO io) {
        this.io = io;
        io.updateInputs(inputs);
        wristPIDController.setSetpoint(0.2);
        wristPIDController.setTolerance(Rotation2d.fromDegrees(0.5).getRotations());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorWrist", inputs);

        // double elevatorPIDValue = elevatorPIDController.calculate(elevatorDistanceTraveled());
        double wristPIDValue = wristPIDController.calculate(inputs.wristAbsoluteEncRawValue);

        // double elevatorFeedForwardValue =
        // elevatorFeedForward.calculate(0, 0, elevatorPIDController.getPeriod());

        // double wristFeedForwardValue =
        // wristFeedForward.calculate(0, 0, wristPIDController.getPeriod());

        // if (inputs.topLimitSwitch && elevatorPIDValue > 0) {
        // elevatorPIDValue = 0;
        // }

        // if (inputs.bottomLimitSwitch && elevatorPIDValue < 0) {
        // elevatorPIDValue = 0;
        // }

        double elevatorPIDValue =
            elevatorPIDController.calculate(-inputs.elevatorRelativeEncRawValue);

        io.setElevatorVoltage(-0.05 * 12.0 - elevatorPIDValue);
        io.setWristVoltage(wristPIDValue);

        // Logger.recordOutput("/ElevatorWrist/Elevator/PID Voltage", elevatorPIDValue);
        // Logger.recordOutput("/ElevatorWrist/Elevator/Feedforward", elevatorFeedForwardValue);
        // Logger.recordOutput("/ElevatorWrist/Elevator/Combined Voltage",
        // elevatorPIDValue + elevatorFeedForwardValue);

        Logger.recordOutput("/ElevatorWrist/Wrist/PID Voltage", elevatorPIDValue);
        Logger.recordOutput("/ElevatorWrist/Wrist/PID setpoint",
            elevatorPIDController.getSetpoint().position);

        SmartDashboard.putNumber("ElevatorWrist PID Voltage", elevatorPIDValue);
        SmartDashboard.putNumber("ElevatorWrist PID setpoint",
            elevatorPIDController.getSetpoint().position);
        SmartDashboard.putNumber("ElevatorWrist Encoder Value", inputs.elevatorRelativeEncRawValue);
        SmartDashboard.putNumber("ElevatorWrist Amp Drawn", inputs.wristMotorAmp);
        // Logger.recordOutput("/ElevatorWrist/Wrist/Feedforward", wristFeedForwardValue);
        // Logger.recordOutput("/ElevatorWrist/Wrist/Combined Voltage",
        // wristFeedForwardValue + wristPIDValue);
        Logger.recordOutput("/ElevatorWrist/Wrist/Combined Voltage", wristPIDValue);

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
            wristPIDController.setSetpoint(angle.getRotations());
        }).andThen(Commands.waitUntil(() -> atGoal()));
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
            // elevatorPIDController.setGoal(height.getAsDouble());
            wristPIDController.setSetpoint(angle.get().getRotations());
        });
    }

    /**
     * Get the height in meters of the elevator based on the rotations of the motor
     *
     * @return Height of elevator in meters
     */
    public double elevatorDistanceTraveled() {
        return inputs.elevatorRelativeEncRawValue
            * Constants.ElevatorWristConstants.SetPoints.LINEAR_DISTANCE;
    }

    /**
     * Get if at elevator + wrist PID goal
     *
     * @return boolean representing if the elevator and wrist PID controllers are at their goals
     */
    public Boolean atGoal() {
        // return elevatorPIDController.atGoal() && wristPIDController.atGoal();
        return wristPIDController.atSetpoint();
        // return true;
    }

    /**
     * Set power output for wrist
     *
     * @param power desired power output percentage
     */
    public void setWristPower(double power) {
        io.setWristPower(power);
    }

    /**
     * Set power output for elevator
     *
     * @param power desired power output percentage
     */
    public void setElevatorPower(double power) {
        io.setElevatorPower(power);
    }

}