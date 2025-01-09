package frc.robot.subsystems.swerve.mod;

import static edu.wpi.first.units.Units.*;
import java.util.Queue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;

public class SwerveModuleTalonDrive implements SwerveModuleDriveIO {

    private TalonFX driveMotor;
    private TalonFXConfiguration driveFXConfig = new TalonFXConfiguration();

    private StatusSignal<Angle> driveMotorPosition;
    private Queue<Double> driveMotorPositionQueue;
    private Queue<Double> timestampQueue;
    private StatusSignal<AngularVelocity> driveMotorVelocity;
    private StatusSignal<Voltage> driveMotorAppliedVolts;
    private StatusSignal<Current> driveMotorCurrent;

    public SwerveModuleTalonDrive(int _id, ModuleConfig config) {
        driveMotor = new TalonFX(config.driveId, "canivore");

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        driveMotorPosition = driveMotor.getPosition();
        driveMotorPositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition());
        driveMotorVelocity = driveMotor.getVelocity();
        driveMotorAppliedVolts = driveMotor.getMotorVoltage();
        driveMotorCurrent = driveMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.Swerve.odometryFrequency,
            driveMotorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveMotorVelocity,
            driveMotorAppliedVolts,
            driveMotorCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveMotor);

        driveFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveFXConfig.Feedback.SensorToMechanismRatio =
            Constants.Swerve.config.moduleConstants.driveReduction;
        driveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveFXConfig.CurrentLimits.SupplyCurrentLimit =
            Constants.Swerve.config.moduleConstants.slipCurrent.in(Amps);
        driveFXConfig.CurrentLimits.SupplyCurrentLowerLimit =
            Constants.Swerve.config.moduleConstants.supplyCurrentLowerLimit.in(Amps);
        driveFXConfig.CurrentLimits.SupplyCurrentLowerTime =
            Constants.Swerve.config.moduleConstants.supplyCurrentLowerTimeThreshold.in(Seconds);

        driveFXConfig.Slot0.kP = Constants.Swerve.config.moduleConstants.drivekP;
        driveFXConfig.Slot0.kI = 0.0;
        driveFXConfig.Slot0.kD = Constants.Swerve.config.moduleConstants.drivekD;
        driveFXConfig.Slot0.kS = Constants.Swerve.config.moduleConstants.ffkS;
        driveFXConfig.Slot0.kV = Constants.Swerve.config.moduleConstants.ffkV;
        driveFXConfig.Slot0.kA = Constants.Swerve.config.moduleConstants.ffkA;

        driveMotor.getConfigurator().apply(driveFXConfig);
        driveMotor.getConfigurator().setPosition(Rotations.of(0.0));
    }

    @Override
    public void updateDriveInputs(SwerveModuleDriveInputs inputs) {
        var driveStatus = BaseStatusSignal.refreshAll(driveMotorPosition);

        inputs.motorConnected = driveStatus.isOK();
        inputs.positionRads = Units.rotationsToRadians(driveMotorPosition.getValueAsDouble());
        inputs.velocityRadsPerSec = Units.rotationsToRadians(driveMotorVelocity.getValueAsDouble());
        inputs.appliedVolts = driveMotorAppliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = driveMotorCurrent.getValueAsDouble();

        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsMeters =
            driveMotorPositionQueue.stream().mapToDouble(Units::rotationsToRadians)
                .map((x) -> x * Constants.Swerve.config.moduleConstants.wheelRadius.in(Meters))
                .toArray();
    }

    @Override
    public void setDrivePID(double p, double i, double d) {
        driveFXConfig.Slot0.kP = p;
        driveFXConfig.Slot0.kI = i;
        driveFXConfig.Slot0.kD = d;
        driveMotor.getConfigurator().apply(driveFXConfig);
    }

    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0).withUpdateFreqHz(0);

    @Override
    public void runDriveOpenLoop(double output) {
        torqueCurrentFOC.Output = output;
        driveMotor.setControl(torqueCurrentFOC);
    }

    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withUpdateFreqHz(0);

    @Override
    public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveVelocity.Velocity = Units.radiansToRotations(velocityRadPerSec);
        driveVelocity.FeedForward = feedforward;
        driveMotor.setControl(driveVelocity);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        driveFXConfig.MotorOutput.NeutralMode =
            enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    }


}
