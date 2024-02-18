package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private int lastAtSetpoint = 0;

    public Shooter(ShooterIO io) {
        this.io = io;
        topPid.setSetpoint(0);
        bottomPid.setSetpoint(0);
        topPid.setTolerance(500);
        bottomPid.setTolerance(500);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        SmartDashboard.putNumber("Shooter Speed Top", inputs.topShooterVelocityRotPerMin);
        SmartDashboard.putNumber("Shooter Speed Bottom", inputs.bottomShooterVelocityRotPerMin);
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
        return inputs.topShooterVelocityRotPerMin;
    }

    public double getBottomVelocity() {
        return inputs.bottomShooterVelocityRotPerMin;
    }

    public double distanceToVelocity(double distance) {
        return 0.0;
    }

    public Boolean atSetpoint() {
        return lastAtSetpoint > 5;
    }

    /**
     * Command to shoot from a distance
     *
     * @return Returns a command
     */
    public Command shootSpeaker() {
        return Commands.runOnce(() -> {
            topPid.setSetpoint(Constants.ShooterConstants.SHOOT_SPEAKER_RPM);
            bottomPid.setSetpoint(Constants.ShooterConstants.SHOOT_SPEAKER_RPM);
            lastAtSetpoint = 0;
        }, this).andThen(Commands.run(() -> {
            setTopMotor(topPid.calculate(inputs.topShooterVelocityRotPerMin)
                + shooterFeed.calculate(Constants.ShooterConstants.SHOOT_SPEAKER_RPM));
            setBottomMotor(bottomPid.calculate(inputs.bottomShooterVelocityRotPerMin)
                + shooterFeed.calculate(Constants.ShooterConstants.SHOOT_SPEAKER_RPM));
            if (topPid.atSetpoint() && bottomPid.atSetpoint()) {
                lastAtSetpoint++;
            } else {
                lastAtSetpoint = 0;
            }
        }, this)).finallyDo(() -> {
            setTopMotor(0);
            setBottomMotor(0);
            lastAtSetpoint = 0;
        });
    }

}
