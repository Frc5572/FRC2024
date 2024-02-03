package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class for Shooter
 */
public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private PIDController pid = new PIDController(Constants.ShooterConstants.KP,
        Constants.ShooterConstants.KI, Constants.ShooterConstants.KD);
    private SimpleMotorFeedforward shooterFeed =
        new SimpleMotorFeedforward(Constants.ShooterConstants.KS, Constants.ShooterConstants.KV);
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();


    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setMotor(double power) {
        Logger.recordOutput("Shooter/Voltage", power);
        io.setMotor(power);
    }

    public double getVelocity() {
        return inputs.shooterVelocityRotPerMin;
    }

    public double distanceToVelocity(double distance) {
        return 0.0;
    }

    public Boolean atSetpoint() {
        return pid.atSetpoint();
    }

    /**
     * Command to shoot from a distance
     *
     * @param distance the distance from the target
     * @return Returns a command
     */
    public Command shootWithDistance(DoubleSupplier distance) {
        return Commands.run(() -> {
            double velocity = distanceToVelocity(distance.getAsDouble());
            setMotor(pid.calculate(getVelocity()) + shooterFeed.calculate(velocity));
        }, this);
    }
}
