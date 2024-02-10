package frc.robot.subsystems.climber;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsystem.
 */
public class Climber extends SubsystemBase {
    public ClimberIO io;
    public ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    PIDController leftClimberPIDController =
        new PIDController(Constants.ClimberConstants.CLIMBER_KP,
            Constants.ClimberConstants.CLIMBER_KI, Constants.ClimberConstants.CLIMBER_KD);
    PIDController rightClimberPIDController =
        new PIDController(Constants.ClimberConstants.CLIMBER_KP,
            Constants.ClimberConstants.CLIMBER_KI, Constants.ClimberConstants.CLIMBER_KD);

    private ElevatorFeedforward climberFeedforward =
        new ElevatorFeedforward(Constants.ClimberConstants.CLIMBER_KS,
            Constants.ClimberConstants.CLIMBER_KG, Constants.ClimberConstants.CLIMBER_KV);

    public Climber(ClimberIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        double leftClimberPIDValue =
            leftClimberPIDController.calculate(leftClimberDistanceTraveled());
        double rightClimberPIDValue =
            rightClimberPIDController.calculate(rightClimberDistanceTraveled());
        double climberFeedForwardValue = climberFeedforward.calculate(0, 0);
        io.setLeftClimberVoltage(climberFeedForwardValue + leftClimberPIDValue);
        io.setRightClimberVoltage(climberFeedForwardValue + rightClimberPIDValue);
        Logger.recordOutput("/Climber/Feedforward", climberFeedForwardValue);
    }

    /**
     * Sets voltage for left side climber system
     *
     * @param power Sets power for climbing motors.
     */
    public void setLeftClimberVoltage(double power) {
        Logger.recordOutput("/Climber/Left/Assigned Voltage", power);
        io.setLeftClimberVoltage(power);

    }

    /**
     * Sets voltage for right side climber system
     *
     * @param power Sets power for climbing motors.
     */
    public void setRightClimberVoltage(double power) {
        Logger.recordOutput("/Climber/Right/Assigned Voltage", power);
        io.setRightClimberVoltage(power);

    }

    /**
     * Climbs to designated position.
     *
     * @param distance Distance to climb
     * @return Returns a usable command
     */
    public Command getToPosition(double distance) {
        BooleanSupplier end =
            () -> leftClimberPIDController.atSetpoint() && rightClimberPIDController.atSetpoint();
        return Commands.runOnce(() -> {
            leftClimberPIDController.setSetpoint(distance);
            rightClimberPIDController.setSetpoint(distance);
        }).andThen(Commands.run(() -> {
            leftClimberPIDController.calculate(leftClimberDistanceTraveled());
            rightClimberPIDController.calculate(leftClimberDistanceTraveled());
            double leftClimberPIDValue =
                leftClimberPIDController.calculate(leftClimberDistanceTraveled());
            double rightClimberPIDValue =
                rightClimberPIDController.calculate(rightClimberDistanceTraveled());
            Logger.recordOutput("/Climber/Left/PID Voltage", leftClimberPIDValue);
            Logger.recordOutput("/Climber/Right/PID Voltage", rightClimberPIDValue);
        }).until(end));

    }

    /**
     * Get the height in meters of the elevator based on the rotations of the motor
     *
     * @return Height of elevator in meters
     */
    public double leftClimberDistanceTraveled() {
        return inputs.leftMotorEncoderValue * Constants.ClimberConstants.LINEAR_DISTANCE;
    }

    /**
     * Get the height in meters of the elevator based on the rotations of the motor
     *
     * @return Height of elevator in meters
     */
    public double rightClimberDistanceTraveled() {
        return inputs.rightMotorEncoderValue * Constants.ClimberConstants.LINEAR_DISTANCE;
    }


}
