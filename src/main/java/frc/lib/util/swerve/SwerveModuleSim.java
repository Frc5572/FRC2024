package frc.lib.util.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class SwerveModuleSim implements SwerveModuleIO {
    public int moduleNumber;
    private FlywheelSim driveSim =
        new FlywheelSim(DCMotor.getFalcon500(1), Constants.Swerve.driveGearRatio, 0.025);
    private FlywheelSim turnSim =
        new FlywheelSim(DCMotor.getFalcon500(1), Constants.Swerve.angleGearRatio, 0.004);

    private double turnRelativePositionRot = 0.0;
    private double turnAbsolutePositionRot = Math.random();
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;


    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
    private PIDController driveFeedback = new PIDController(0.1, 0.0, 0.0);
    private PIDController turnFeedback = new PIDController(0.1, 0.0, 0.0);

    public SwerveModuleSim() {
        System.out.println("[Init] Creating ServeModuleSim");
        turnFeedback.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void setModNumber(int number) {
        this.moduleNumber = number;
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(Constants.loopPeriodSecs);
        turnSim.update(Constants.loopPeriodSecs);
        double angleDiffRot = Units
            .radiansToRotations(turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
        turnRelativePositionRot += angleDiffRot;
        turnAbsolutePositionRot += angleDiffRot;
        while (turnAbsolutePositionRot < 0) {
            turnAbsolutePositionRot += 1;
        }
        while (turnAbsolutePositionRot > 1) {
            turnAbsolutePositionRot -= 1;
        }

        inputs.driveMotorSelectedPosition =
            inputs.driveMotorSelectedPosition + Units.radiansToRotations(
                (driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs));
        inputs.driveMotorSelectedSensorVelocity =
            Units.radiansPerSecondToRotationsPerMinute(driveSim.getAngularVelocityRadPerSec());

        inputs.angleMotorSelectedPosition = turnRelativePositionRot;

        inputs.absolutePositionAngleEncoder = turnAbsolutePositionRot;
        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    }

    public void setDriveMotor(double mps) {
        double rpm = Conversions.metersPerSecondToRotationPerSecond(mps,
            Constants.Swerve.wheelCircumference);
        driveFeedback.setSetpoint(rpm);
        double driveFF = driveFeedforward.calculate(mps);
        SmartDashboard.putNumber("ff/" + moduleNumber, driveFF);
        double volts = driveFeedback.calculate(mps) + driveFF;
        if (rpm == 0) {
            volts = 0;
        }
        SmartDashboard.putNumber("Drive volts/" + moduleNumber, volts);
        setDriveVoltage(volts);
    }

    public void setAngleMotor(double angle) {
        turnFeedback.setSetpoint(angle);
        double volts = turnFeedback.calculate(turnAbsolutePositionRot);
        SmartDashboard.putNumber("Angle volts/" + moduleNumber, volts);
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
