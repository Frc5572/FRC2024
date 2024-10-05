package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Class for ShooterSim
 */
public class ShooterSim implements ShooterIO {

    private FlywheelSim topShooterMotor =
        new FlywheelSim(DCMotor.getNeoVortex(1), 1 / Constants.ShooterConstants.GEAR_RATIO, 0.025);
    private FlywheelSim bottomShooterMotor =
        new FlywheelSim(DCMotor.getNeoVortex(1), 1 / Constants.ShooterConstants.GEAR_RATIO, 0.025);

    private double topAppliedVolts = 0.0;
    private double bottomAppliedVolts = 0.0;


    /**
     * Constructor Shooter Subsystem - sets motor and encoder preferences
     */
    public ShooterSim() {}

    public void setTopMotor(double power) {
        topAppliedVolts = MathUtil.clamp(power, -12.0, 12.0);
        topShooterMotor.setInputVoltage(topAppliedVolts);
    }

    public void setBottomMotor(double power) {
        bottomAppliedVolts = MathUtil.clamp(power, -12.0, 12.0);
        bottomShooterMotor.setInputVoltage(bottomAppliedVolts);
    }


    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        topShooterMotor.update(LoggedRobot.defaultPeriodSecs);
        bottomShooterMotor.update(LoggedRobot.defaultPeriodSecs);
        inputs.topShooterVelocityRotPerMin = topShooterMotor.getAngularVelocityRPM();
        inputs.bottomShooterVelocityRotPerMin = bottomShooterMotor.getAngularVelocityRPM();
    }
}
