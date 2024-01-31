package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsystem.
 */
public class Climber extends SubsystemBase {
    public ClimberIO io;
    public ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    PIDController climberPID = new PIDController(Constants.ClimberConstants.CLIMBER_KP,
        Constants.ClimberConstants.CLIMBER_KI, Constants.ClimberConstants.CLIMBER_KD);

    private ElevatorFeedforward climbFeedforward =
        new ElevatorFeedforward(Constants.ClimberConstants.CLIMBER_KS,
            Constants.ClimberConstants.CLIMBER_KG, Constants.ClimberConstants.CLIMBER_KV);

    public Climber(ClimberIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        Logger.processInputs("Climber", inputs);
        io.updateInputs(inputs);
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


}
