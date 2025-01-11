package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.sim.SimulatedPumbaa;
import frc.robot.Constants;

/**
 * Class for ShooterSim
 */
public class ShooterSim implements ShooterIO {

    private FlywheelSim topShooterMotor =
    new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), 1, Constants.ShooterConstants.GEAR_RATIO), DCMotor.getNeoVortex(2), 0.0);
        //new FlywheelSim(DCMotor.getNeoVortex(1), Constants.ShooterConstants.GEAR_RATIO, 0.01);
    private FlywheelSim bottomShooterMotor =
    new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 1, Constants.ShooterConstants.GEAR_RATIO), DCMotor.getNeoVortex(1), 0.0);
    
       // new FlywheelSim(DCMotor.getNeoVortex(1), Constants.ShooterConstants.GEAR_RATIO, 0.01);

    private double topAppliedVolts = 0.0;
    private double bottomAppliedVolts = 0.0;

    private final SimulatedPumbaa sim;

    /**
     * Constructor Shooter Subsystem - sets motor and encoder preferences
     */
    public ShooterSim(SimulatedPumbaa sim) {
        this.sim = sim;
    }

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
        sim.setShooterSpeed(topShooterMotor.getAngularVelocityRPM());
    }
}
