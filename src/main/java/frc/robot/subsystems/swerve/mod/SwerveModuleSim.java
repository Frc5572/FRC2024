package frc.robot.subsystems.swerve.mod;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class SwerveModuleSim implements SwerveModuleAngleIO, SwerveModuleDriveIO {

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final PIDController driveController;
    private final PIDController turnController;
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SwerveModuleSim(ModuleConfig config, SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = moduleSimulation.useGenericMotorControllerForDrive()
            .withCurrentLimit(Constants.Swerve.slipCurrent);
        this.turnMotor =
            moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Units.Amps.of(20));

        this.driveController = new PIDController(0.05, 0.0, 0.0);
        this.turnController = new PIDController(8.0, 0.0, 0.0);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateDriveInputs(SwerveModuleDriveInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController
                .calculate(moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond));
        } else {
            driveController.reset();
        }

        driveMotor.requestVoltage(Units.Volts.of(driveAppliedVolts));

        inputs.motorConnected = true;
        inputs.positionRads = moduleSimulation.getDriveWheelFinalPosition().in(Units.Radians);
        inputs.velocityRadsPerSec =
            moduleSimulation.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond);
        inputs.appliedVolts = driveAppliedVolts;
        inputs.supplyCurrentAmps =
            Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Units.Amps));
        inputs.odometryDrivePositionsMeters =
            new double[] {inputs.positionRads * Constants.Swerve.wheelRadius.in(Units.Meters)};
    }

    @Override
    public void updateAngleInputs(SwerveModuleAngleInputs inputs) {
        if (turnClosedLoop) {
            turnAppliedVolts =
                turnController.calculate(moduleSimulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turnController.reset();
        }

        turnMotor.requestVoltage(Units.Volts.of(turnAppliedVolts));

        inputs.motorConnected = true;
        inputs.encoderConnected = true;
        inputs.absolutePosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.position = moduleSimulation.getSteerAbsoluteFacing();
        inputs.velocityRadsPerSec =
            moduleSimulation.getSteerAbsoluteEncoderSpeed().in(Units.RadiansPerSecond);
        inputs.appliedVolts = turnAppliedVolts;
        inputs.supplyCurrentAmps =
            Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Units.Amps));
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.position};
    }

}
