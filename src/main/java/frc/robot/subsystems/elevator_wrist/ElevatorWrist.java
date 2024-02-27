package frc.robot.subsystems.elevator_wrist;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/**
 * Elevator and Wrist Subsystem
 */
public class ElevatorWrist extends SubsystemBase {
    public ElevatorWristIO io;
    public ElevatorWristInputsAutoLogged inputs = new ElevatorWristInputsAutoLogged();
    private CommandXboxController operator;

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

    PIDController wristProfiledPIDController =
        new PIDController(Constants.ElevatorWristConstants.PID.WRIST_LARGE_KP,
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

    /**
     * Create new ElevatorWrist.
     */
    public ElevatorWrist(ElevatorWristIO io, CommandXboxController operator) {
        this.operator = operator;
        this.io = io;
        io.updateInputs(inputs);
        wristPIDController
            .setSetpoint(Constants.ElevatorWristConstants.SetPoints.AMP_ANGLE.getRotations());
        wristPIDController.setTolerance(Rotation2d.fromDegrees(0.1).getRotations());
        elevatorPIDController.setGoal(36);
        wristPIDController.setIZone(Rotation2d.fromDegrees(5).getRotations());
        wristProfiledPIDController.setIZone(Rotation2d.fromDegrees(1).getRotations());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorWrist", inputs);



        if (inputs.wristAbsoluteEncRawValue > 0.5) {
            inputs.wristAbsoluteEncRawValue -= 1.0;
        }

        wristProfiledPIDController.setSetpoint(wristPIDController.getSetpoint());

        // double elevatorPIDValue = elevatorPIDController.calculate(elevatorDistanceTraveled());

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

        Rotation2d calculatedWristAngle = getWristAngle();

        double calculatedHeight = getHeight();

        SmartDashboard.putNumber("calculatedHeight", calculatedHeight);
        SmartDashboard.putNumber("calculatedWristAngle", calculatedWristAngle.getDegrees());

        double wristPIDValue = wristPIDController.calculate(calculatedWristAngle.getRotations());

        double profiledWristPIDValue =
            wristProfiledPIDController.calculate(calculatedWristAngle.getRotations());
        SmartDashboard.putNumber("wristError",
            Rotation2d.fromRotations(wristPIDController.getPositionError()).getDegrees());
        if (Math.abs(wristPIDController.getPositionError()) > Rotation2d.fromDegrees(5)
            .getRotations()) {
            wristPIDValue = profiledWristPIDValue;
        }

        double elevatorPIDValue = elevatorPIDController.calculate(calculatedHeight);
        double elevatorFeedForward = 0.4;

        if (operator.a().getAsBoolean()) {
            if (calculatedHeight > 24.7 && calculatedHeight < 30) { // Getting around a bearing that
                // prevents us from moving without
                // considerable effort
                if (elevatorPIDValue < 0.0) {
                    elevatorFeedForward = -0.6;
                } else {
                    elevatorFeedForward = 3.0;
                }
            }
            io.setElevatorVoltage(-elevatorFeedForward - elevatorPIDValue);
            io.setWristVoltage(-wristPIDValue);
        } else {
            io.setWristVoltage(operator.getLeftY() * 4.0);
            io.setElevatorVoltage(-elevatorFeedForward + operator.getRightY() * 4.0);
        }

        // Logger.recordOutput("/ElevatorWrist/Elevator/PID Voltage", elevatorPIDValue);
        // Logger.recordOutput("/ElevatorWrist/Elevator/Feedforward", elevatorFeedForwardValue);
        // Logger.recordOutput("/ElevatorWrist/Elevator/Combined Voltage",
        // elevatorPIDValue + elevatorFeedForwardValue);

        Logger.recordOutput("/ElevatorWrist/Wrist/PID Voltage", elevatorPIDValue);
        Logger.recordOutput("/ElevatorWrist/Wrist/PID setpoint",
            elevatorPIDController.getSetpoint().position);

        SmartDashboard.putNumber("Wrist Goal",
            Rotation2d.fromRotations(wristPIDController.getSetpoint()).getDegrees());
        SmartDashboard.putNumber("Wrist Profiled Goal",
            Rotation2d.fromRotations(wristProfiledPIDController.getSetpoint()).getDegrees());
        SmartDashboard.putNumber("Elevator PID Voltage", elevatorPIDValue);
        SmartDashboard.putNumber("Wrist PID Voltage", wristPIDValue);
        SmartDashboard.putNumber("ElevatorWrist PID setpoint",
            elevatorPIDController.getSetpoint().position);
        SmartDashboard.putNumber("ElevatorWrist Elevator Encoder Value",
            inputs.elevatorRelativeEncRawValue);
        SmartDashboard.putNumber("ElevatorWrist Wrist Encoder Value",
            inputs.wristAbsoluteEncRawValue);
        SmartDashboard.putNumber("ElevatorWrist Amp Drawn", inputs.wristMotorAmp);
        // Logger.recordOutput("/ElevatorWrist/Wrist/Feedforward", wristFeedForwardValue);
        // Logger.recordOutput("/ElevatorWrist/Wrist/Combined Voltage",
        // wristFeedForwardValue + wristPIDValue);
        Logger.recordOutput("/ElevatorWrist/Wrist/Combined Voltage", wristPIDValue);

    }

    /**
     * Calculate elevator height from raw encoder value.
     */
    public double getHeight() {
        return Constants.ElevatorWristConstants.ELEVATOR_M * inputs.elevatorRelativeEncRawValue
            + Constants.ElevatorWristConstants.ELEVATOR_B;
    }

    /**
     * Calculate wrist angle from raw encoder value.
     */
    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(
            Constants.ElevatorWristConstants.WRIST_M * inputs.wristAbsoluteEncRawValue
                + Constants.ElevatorWristConstants.WRIST_B);
    }

    /**
     * Set elevator and wrist to amp position. Performs two steps to avoid colliding with
     * electronics box.
     */
    public Command ampPosition() {
        return goToPosition(Constants.ElevatorWristConstants.SetPoints.AMP_HEIGHT,
            Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE).until(() -> getHeight() > 32)
                .withTimeout(2)
                .andThen(goToPosition(Constants.ElevatorWristConstants.SetPoints.AMP_HEIGHT,
                    Constants.ElevatorWristConstants.SetPoints.AMP_ANGLE).withTimeout(0.5));
    }

    /**
     * Set elevator to home position which can fit under the stage. Performs two steps to avoid
     * colliding with electronics box.
     */
    public Command homePosition() {
        return goToPosition(36, Rotation2d.fromDegrees(24))
            .until(() -> getWristAngle().getDegrees() > 15).withTimeout(2)
            .andThen(goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE).withTimeout(0.5));
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
            wristProfiledPIDController.setSetpoint(angle.getRotations());
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
            wristProfiledPIDController.setSetpoint(angle.get().getRotations());
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
