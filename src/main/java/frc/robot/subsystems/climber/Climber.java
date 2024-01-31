package frc.robot.subsystems.climber;

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
        new PIDController(Constants.ClimberConstants.LEFT_CLIMBER_KP,
            Constants.ClimberConstants.LEFT_CLIMBER_KI, Constants.ClimberConstants.LEFT_CLIMBER_KD);
    PIDController rightClimberPIDController = new PIDController(
        Constants.ClimberConstants.RIGHT_CLIMBER_KP, Constants.ClimberConstants.RIGHT_CLIMBER_KI,
        Constants.ClimberConstants.RIGHT_CLIMBER_KD);

    private ElevatorFeedforward leftClimberFeedforward =
        new ElevatorFeedforward(Constants.ClimberConstants.LEFT_CLIMBER_KS,
            Constants.ClimberConstants.LEFT_CLIMBER_KG, Constants.ClimberConstants.LEFT_CLIMBER_KV);
    private ElevatorFeedforward rightClimberFeedforward = new ElevatorFeedforward(
        Constants.ClimberConstants.RIGHT_CLIMBER_KS, Constants.ClimberConstants.RIGHT_CLIMBER_KG,
        Constants.ClimberConstants.RIGHT_CLIMBER_KV);

    public Climber(ClimberIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        // Dont know where to put it rn

        // double leftClimberPIDValue =
        // leftClimberPIDController.calculate(leftClimberDistanceTraveled());
        // double rightClimberPIDValue =
        // rightClimberPIDController.calculate(rightClimberDistanceTraveled());
        // Logger.recordOutput("/Climber/VoltageFromPID/LeftClimber", leftClimberPIDValue);
        // Logger.recordOutput("/Climber/VoltageFromPID/RightClimber", rightClimberPIDValue);

        double leftClimberFeedForwardValue =
            leftClimberFeedforward.calculate(0, 0, leftClimberPIDController.getPeriod());
        double rightClimberFeedForwardValue =
            rightClimberFeedforward.calculate(0, 0, rightClimberPIDController.getPeriod());

        Logger.recordOutput("/Climber/VoltageFromFeedForward/LeftClimber",
            leftClimberFeedForwardValue);
        Logger.recordOutput("/Climber/VoltageFromFeedForward/RightClimber",
            rightClimberFeedForwardValue);
    }

    /**
     * Sets voltage for climber system
     *
     * @param power Sets power for climbing motors.
     */
    public void setClimberVoltage(double power) {
        Logger.recordOutput("/Climber/Voltage", power);
        io.setClimberVoltage(power);

    }

    /**
     * Climbs to designated position.
     *
     * @param distance Distance to climb
     * @return Returns a useable command
     */
    public Command getToPosition(double distance) {
        return Commands.runOnce(() -> {
            leftClimberPIDController.setSetpoint(distance);
            rightClimberPIDController.setSetpoint(distance);
        }).andThen(Commands.waitUntil(
            () -> leftClimberPIDController.atSetpoint() && rightClimberPIDController.atSetpoint()));
    }

    /**
     * Get the height in meters of the elevator based on the rotations of the motor
     *
     * @return Height of elevator in meters
     */
    public double leftClimberDistanceTraveled() {
        return inputs.leftMotorEncoderValue;
    }

    /**
     * Get the height in meters of the elevator based on the rotations of the motor
     *
     * @return Height of elevator in meters
     */
    public double rightClimberDistanceTraveled() {
        return inputs.rightMotorEncoderValue;
    }


}
