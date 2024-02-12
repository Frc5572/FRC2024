package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RPM.of(0));

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
        return inputs.topShooterVelocityRotPerMin;
    }

    public double getBottomVelocity() {
        return inputs.bottomShooterVelocityRotPerMin;
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
            double velocity = 1000 / 60; // distanceToVelocity(distance.getAsDouble());
            setTopMotor(pid.calculate(getTopVelocity()) + shooterFeed.calculate(velocity));
            setBottomMotor(pid.calculate(getBottomVelocity()) + shooterFeed.calculate(velocity));
        }, this);
    }

    // Create a new SysId routine for characterizing the shooter.
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
        new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motor(s).
            (Measure<Voltage> volts) -> {
                // io.setTopMotor(volts.in(Volts));
                io.setBottomMotor(volts.in(Volts));

            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
                // Record a frame for the shooter motor.
                // log.motor("top-shooter")
                // .voltage(m_appliedVoltage.mut_replace(inputs.topShooterSupplyVoltage, Volts))
                // .angularPosition(m_angle.mut_replace(inputs.topShooterPosition, Rotations))
                // .angularVelocity(
                // m_velocity.mut_replace(inputs.topShooterVelocityRotPerMin, RPM));
                log.motor("bottom-shooter")
                    .voltage(m_appliedVoltage.mut_replace(
                        inputs.bottomShooterPower * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(inputs.bottomShooterPosition, Rotations))
                    .angularVelocity(m_velocity.mut_replace(
                        inputs.bottomShooterVelocityRotPerMin / 60, RotationsPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name ("shooter")
            this));

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
