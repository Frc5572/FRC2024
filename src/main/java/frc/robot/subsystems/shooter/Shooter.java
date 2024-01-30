package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class for Shooter
 */
public class Shooter extends SubsystemBase {
    private ShooterIO io;
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

    public double distanceToVelocity(double distance) {
        return 0.0;
    }

    public Command shootWithDistance(DoubleSupplier distance) {
        return Commands.run(() -> {
            double velocity = distanceToVelocity(distance.getAsDouble());
            setTopMotor(velocity);
            setBottomMotor(velocity);
        }, this);
    }
}
