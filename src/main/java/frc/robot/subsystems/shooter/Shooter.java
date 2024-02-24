package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.ReconfigurableSimpleMotorFeedforward;
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
    private ReconfigurableSimpleMotorFeedforward bottomShooterFeed =
        new ReconfigurableSimpleMotorFeedforward(Constants.ShooterConstants.KS,
            Constants.ShooterConstants.BOTTOM_KV);
    private ReconfigurableSimpleMotorFeedforward topShooterFeed =
        new ReconfigurableSimpleMotorFeedforward(Constants.ShooterConstants.KS,
            Constants.ShooterConstants.TOP_KV);
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double lastAtSetpoint = Timer.getFPGATimestamp();

    private double topValue;
    private double bottomValue;

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

        SmartDashboard.putNumber("Top Target", topPid.getSetpoint());
        SmartDashboard.putNumber("Top Current", inputs.topShooterVelocityRotPerMin);
        SmartDashboard.putNumber("Top Error", topPid.getPositionError());
        SmartDashboard.putNumber("Top Value", topValue);

        SmartDashboard.putNumber("Bottom Target", bottomPid.getSetpoint());
        SmartDashboard.putNumber("Bottom Current", inputs.bottomShooterVelocityRotPerMin);
        SmartDashboard.putNumber("Bottom Error", bottomPid.getPositionError());
        SmartDashboard.putNumber("Bottom Value", bottomValue);

        topShooterFeed.kv = SmartDashboard.getNumber("topShooterFeed", topShooterFeed.kv);
        bottomShooterFeed.kv = SmartDashboard.getNumber("bottomShooterFeed", bottomShooterFeed.kv);
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

    /*
     * 
     * Set voltage to Bottom Shooter Motor**
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
        return lastAtSetpoint < Timer.getFPGATimestamp() - 0.2;
    }

    /**
     * Set the shooter RPM setpoint
     *
     * @param setpoint Setpoint in RPM
     */
    public void setSetpoint(double setpoint) {
        topPid.setSetpoint(setpoint);
        bottomPid.setSetpoint(setpoint);
        lastAtSetpoint = Timer.getFPGATimestamp();
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
                topValue = topPid.calculate(inputs.topShooterVelocityRotPerMin)
                    + topShooterFeed.calculate(Constants.ShooterConstants.SHOOT_SPEAKER_RPM);
                setTopMotor(topValue);
                bottomValue = bottomPid.calculate(inputs.bottomShooterVelocityRotPerMin)
                    + bottomShooterFeed.calculate(Constants.ShooterConstants.SHOOT_SPEAKER_RPM);
                setBottomMotor(bottomValue);
                if (!atSetpoint()) {
                    lastAtSetpoint = Timer.getFPGATimestamp();
                }
            }, this)).finallyDo(() -> {
                setTopMotor(0);
                setBottomMotor(0);
                lastAtSetpoint = Timer.getFPGATimestamp();
            });
    }
}
