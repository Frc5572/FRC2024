package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class for Shooter
 */
public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private PIDController pid = new PIDController(Constants.ShooterConstants.kP,
        Constants.ShooterConstants.kI, Constants.ShooterConstants.kD);
    private SimpleMotorFeedforward shooterFeed =
        new SimpleMotorFeedforward(Constants.ShooterConstants.kS, Constants.ShooterConstants.kV);
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setTopMotor(double power) {
        Logger.recordOutput("Shooter/Top Voltage", power);
        io.setTopMotor(power);
    }

    public void setBottomMotor(double power) {
        Logger.recordOutput("Shooter/Bottom Voltage", power);
        io.setBottomMotor(power);
    }

    public double getTopVelocity() {
        return inputs.topshooterVelocityRotPerSecond;
    }

    public double getBottomVelocity() {
        return inputs.bottomshooterVelocityRotPerSecond;
    }
}
