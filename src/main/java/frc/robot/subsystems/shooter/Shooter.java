package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private PIDController pid =
        new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        double VoltageOutput = pid.calculate(inputs.shooterVelocityRotPerSecond);
        io.setTopMotor(VoltageOutput);
        io.setBottomMotor(VoltageOutput);
    }

    public void setTopMotor(double power) {
        Logger.recordOutput("Top Shooter/Voltage", power);
        io.setTopMotor(power);
    }

    public void setBottomMotor(double power) {
        Logger.recordOutput("Bottom Shooter/Voltage", power);
        io.setBottomMotor(power);
    }
}
