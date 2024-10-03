package frc.lib.util.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModuleSim implements SwerveModuleIO {
    public int moduleNumber;
    private FlywheelSim driveSim =
        new FlywheelSim(DCMotor.getFalcon500(1), Constants.Swerve.driveGearRatio, 0.025);
    private FlywheelSim turnSim =
        new FlywheelSim(DCMotor.getFalcon500(1), Constants.Swerve.angleGearRatio, 0.004);

    private final Rotation2d turnAbsoluteInitPosition =
        new Rotation2d(Math.random() * 2.0 * Math.PI);

    private double turnRelativePositionRad = 0.0;
    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private final PIDController driveFeedback;
    private final PIDController turnFeedback;

    public SwerveModuleSim(int moduleNumber) {
        this.moduleNumber = moduleNumber;
        System.out.println("[Init] Creating ServeModuleSim");
        driveFeedback = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI,
            Constants.Swerve.driveKD);
        turnFeedback = new PIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI,
            Constants.Swerve.angleKD);
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        turnSim.update(Constants.loopPeriodSecs);

        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.driveMotorSelectedPosition = Rotation2d
            .fromRadians(inputs.driveMotorSelectedPosition
                + (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs))
            .getRotations();
        inputs.driveMotorSelectedSensorVelocity =
            Rotation2d.fromRadians(driveSim.getAngularVelocityRadPerSec()).getRotations();

        inputs.angleMotorSelectedPosition =
            Rotation2d.fromRadians(turnAbsolutePositionRad).getRotations();
        inputs.absolutePositionAngleEncoder = 0;
    }

    public void setDriveMotor(double rpm, double feedforward) {
        driveFeedback.setSetpoint(rpm);
        double volts =
            driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec() / (2 * Math.PI))
                + feedforward;
        SmartDashboard.putNumber("rpm/" + moduleNumber, rpm);
        SmartDashboard.putNumber("ff/" + moduleNumber, feedforward);
        SmartDashboard.putNumber("volts/" + moduleNumber, volts);
        setDriveVoltage(volts);
    }

    public void setAngleMotor(double angle) {
        turnFeedback.setSetpoint(angle);
        double volts = turnFeedback.calculate(turnAbsolutePositionRad / (2 * Math.PI));
        setTurnVoltage(volts);
    }

    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }
}
