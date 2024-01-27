package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    // would have to calculate pid values based on distance off of ground. minimum distance 23
    // inches, max 50 inches. measure value using relative encoder ticks

    // revolutions -> distance = (revolutions / gear ratio) * circumference or pi*diameter
    public void setClimberVoltage() {
        io.setClimberVoltage(climberPID.calculate());
    }


}
