package frc.robot.subsystems.shooter;

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
    private int lastAtSetpoint = 0;

    /**
     * Shooter Subsystem
     *
     * @param io Shooter IO Layer
     */
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
    }

    /**
     * Set voltage to Top Shooter Motor
     *
     * @param voltage Voltage to apply to motor
     */
    public void setTopMotor(double voltage) {
        Logger.recordOutput("Shooter/Top Voltage", voltage);
        io.setTopMotor(voltage);
    }

    /**
     * Set voltage to Bottom Shooter Motor
     *
     * @param voltage Voltage to apply to motor
     */
    public void setBottomMotor(double voltage) {
        Logger.recordOutput("Shooter/Bottom Voltage", voltage);
        io.setBottomMotor(voltage);
    }

    /**
     * If Both shooter motors are currently at their setpoint within the tolerance
     *
     * @return True if at setpoint
     */
    public Boolean atSetpoint() {
        return topPid.atSetpoint() && bottomPid.atSetpoint();
    }

    /**
     * If Both shooter motors have a consistent RPM within their Setpoint tolerance for more than 5
     * cycles
     *
     * @return True if at setpoint
     */
    public Boolean readyToShoot() {
        return lastAtSetpoint > 5;
    }

    /**
     * Set the shooter RPM setpoint
     *
     * @param setpoint Setpoint in RPM
     */
    public void setSetpoint(double setpoint) {
        topPid.setSetpoint(setpoint);
        bottomPid.setSetpoint(setpoint);
        lastAtSetpoint = 0;
    }

    /**
     * Command to run the shooter at a set RPM for the speaker
     *
     * @return Returns a command
     */
    public Command shootSpeaker() {
        return Commands
            .runOnce(() -> setSetpoint(Constants.ShooterConstants.SHOOT_SPEAKER_RPM), this)
            .andThen(Commands.run(() -> {
                setTopMotor(topPid.calculate(inputs.topShooterVelocityRotPerMin)
                    + shooterFeed.calculate(Constants.ShooterConstants.SHOOT_SPEAKER_RPM));
                setBottomMotor(bottomPid.calculate(inputs.bottomShooterVelocityRotPerMin)
                    + shooterFeed.calculate(Constants.ShooterConstants.SHOOT_SPEAKER_RPM));
                if (atSetpoint()) {
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
