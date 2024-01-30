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
    private PIDController frontpid = new PIDController(Constants.ShooterConstants.KP,
        Constants.ShooterConstants.KI, Constants.ShooterConstants.KD);
    private PIDController backpid = new PIDController(Constants.ShooterConstants.KP,
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

    public void setFrontMotor(double power) {
        Logger.recordOutput("Shooter/Top Voltage", power);
        io.setTopMotor(power);
    }

    public void setBackMotor(double power) {
        Logger.recordOutput("Shooter/Bottom Voltage", power);
        io.setBottomMotor(power);
    }

    public double getFrontVelocity() {
        return inputs.topshooterVelocityRotPerSecond;
    }

    public double getBackVelocity() {
        return inputs.bottomshooterVelocityRotPerSecond;
    }

    public double distanceToVelocity(double distance) {
        return 0.0;
    }

    public Command shootWithDistance(DoubleSupplier distance) {
        return Commands.run(() -> {
            double velocity = distanceToVelocity(distance.getAsDouble());
            setFrontMotor(frontpid.calculate(getFrontVelocity()) + shooterFeed.calculate(velocity));
            setBackMotor(backpid.calculate(getBackVelocity()) + shooterFeed.calculate(velocity));
        }, this);
    }
}
