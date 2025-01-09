package frc.robot.subsystems.swerve.mod;

import static edu.wpi.first.units.Units.*;
import java.util.Queue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.PhoenixOdometryThread;

public class SwerveModuleTalonAngle implements SwerveModuleAngleIO {

    private CANcoder encoder;
    private TalonFX angleMotor;
    private TalonFXConfiguration angleFXConfig = new TalonFXConfiguration();
    private CANcoderConfiguration angleEncConfig = new CANcoderConfiguration();

    private StatusSignal<Angle> angleAbsolutePosition;
    private StatusSignal<Angle> angleMotorPosition;
    private Queue<Double> angleMotorPositionQueue;
    private StatusSignal<AngularVelocity> angleMotorVelocity;
    private StatusSignal<Voltage> angleMotorAppliedVolts;
    private StatusSignal<Current> angleMotorCurrent;

    private final Rotation2d encoderOffset;

    public SwerveModuleTalonAngle(int _id, ModuleConfig config) {
        angleMotor = new TalonFX(config.angleId, "canivore");
        encoder = new CANcoder(config.absoluteEncoderId, "canivore");
        encoderOffset = config.absoluteEncoderOffset;

        angleAbsolutePosition = encoder.getAbsolutePosition();
        angleMotorPosition = angleMotor.getPosition();
        angleMotorPositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(angleMotor.getPosition());
        angleMotorVelocity = angleMotor.getVelocity();
        angleMotorAppliedVolts = angleMotor.getMotorVoltage();
        angleMotorCurrent = angleMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            Constants.Swerve.odometryFrequency,
            angleMotorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            angleMotorVelocity,
            angleMotorAppliedVolts,
            angleMotorCurrent,
            angleAbsolutePosition);
        ParentDevice.optimizeBusUtilizationForAll(angleMotor);

        angleFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        angleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleFXConfig.Feedback.SensorToMechanismRatio =
            Constants.Swerve.config.moduleConstants.angleReduction;
        angleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        angleFXConfig.MotorOutput.Inverted =
            config.angleMotorInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        angleFXConfig.Slot0.kP = Constants.Swerve.config.moduleConstants.anglekP;
        angleFXConfig.Slot0.kI = 0.0;
        angleFXConfig.Slot0.kD = Constants.Swerve.config.moduleConstants.anglekD;

        angleMotor.getConfigurator().apply(angleFXConfig);
        angleMotor.getConfigurator().setPosition(Rotations.of(0.0));

        angleEncConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
        encoder.getConfigurator().apply(angleEncConfig);
    }

    @Override
    public void updateAngleInputs(SwerveModuleAngleInputs inputs) {
        var angleStatus =
            BaseStatusSignal.refreshAll(angleMotorPosition, angleMotorVelocity,
                angleMotorAppliedVolts, angleMotorCurrent);
        var angleEncoderStatus = BaseStatusSignal.refreshAll(angleAbsolutePosition);

        inputs.motorConnected = angleStatus.isOK();
        inputs.encoderConnected =
            angleEncoderStatus.isOK();
        inputs.absolutePosition =
            Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble()).minus(encoderOffset);
        inputs.position = Rotation2d.fromRotations(angleMotorPosition.getValueAsDouble());
        inputs.velocityRadsPerSec = Units.rotationsToRadians(angleMotorVelocity.getValueAsDouble());
        inputs.appliedVolts = angleMotorAppliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = angleMotorCurrent.getValueAsDouble();

        inputs.odometryTurnPositions = angleMotorPositionQueue.stream()
            .map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        angleMotorPositionQueue.clear();
    }

    @Override
    public void setAnglePID(double p, double i, double d) {
        angleFXConfig.Slot0.kP = p;
        angleFXConfig.Slot0.kI = i;
        angleFXConfig.Slot0.kD = d;
        angleMotor.getConfigurator().apply(angleFXConfig);
    }

    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0).withUpdateFreqHz(0);

    @Override
    public void runAngleOpenLoop(double output) {
        torqueCurrentFOC.Output = output;
        angleMotor.setControl(torqueCurrentFOC);
    }

    private final PositionVoltage positionVoltage =
        new PositionVoltage(0).withSlot(0).withUpdateFreqHz(0);

    @Override
    public void runAnglePosition(Rotation2d rotation) {
        positionVoltage.Position = rotation.getRotations();
        angleMotor.setControl(positionVoltage);
    }

}
