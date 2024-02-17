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
    private PIDController topPid = new PIDController(Constants.ShooterConstants.KP,
        Constants.ShooterConstants.KI, Constants.ShooterConstants.KD);
    private PIDController bottomPid = new PIDController(Constants.ShooterConstants.KP,
        Constants.ShooterConstants.KI, Constants.ShooterConstants.KD);
    private SimpleMotorFeedforward shooterFeed =
        new SimpleMotorFeedforward(Constants.ShooterConstants.KS, Constants.ShooterConstants.KV);
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private boolean shooterIsOn;

    public Shooter(ShooterIO io) {
        this.io = io;
        shooterIsOn = false;
        topPid.setSetpoint(Constants.ShooterConstants.DESIRED_SPEED);
        bottomPid.setSetpoint(Constants.ShooterConstants.DESIRED_SPEED);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        if (shooterIsOn) {
            setTopMotor(topPid.calculate(inputs.topShooterVelocityRotPerMin)
                + shooterFeed.calculate(Constants.ShooterConstants.DESIRED_SPEED));
            setBottomMotor(bottomPid.calculate(inputs.bottomShooterVelocityRotPerMin)
                + shooterFeed.calculate(Constants.ShooterConstants.DESIRED_SPEED));
        } else {
            setTopMotor(0);
            setBottomMotor(0);
        }
    }

    public void setTopMotor(double power) {
        Logger.recordOutput("Shooter/Top Voltage", power);
        io.setTopMotor(power);
    }

    public void setBottomMotor(double power) {
        Logger.recordOutput("Shooter/Bottom Voltage", power);
        io.setBottomMotor(power);
    }

    public void setActive(boolean active) {
        this.shooterIsOn = active;
    }

    public double getTopVelocity() {
        return inputs.topShooterVelocityRotPerMin;
    }

    public double getBottomVelocity() {
        return inputs.bottomShooterVelocityRotPerMin;
    }

    public double distanceToVelocity(double distance) {
        return 0.0;
    }

    public Boolean atSetpoint() {
        return topPid.atSetpoint() && bottomPid.atSetpoint();
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
            // double velocity = 1000 / 60; // distanceToVelocity(distance.getAsDouble());
            setTopMotor(topPid.calculate(getTopVelocity()) + shooterFeed.calculate(velocity));
            setBottomMotor(
                bottomPid.calculate(getBottomVelocity()) + shooterFeed.calculate(velocity));
        }, this);
    }

}
