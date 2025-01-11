package frc.lib.util.swerve;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.lib.math.Conversions;
import frc.robot.Constants;

/**
 * Swerve Module IO
 */
public class SwerveModuleReal implements SwerveModuleIO {

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    private TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    private CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    private StatusSignal<Angle> driveMotorSelectedPosition;
    private StatusSignal<AngularVelocity> driveMotorSelectedSensorVelocity;
    private StatusSignal<Angle> angleMotorSelectedPosition;
    private StatusSignal<Angle> absolutePositionAngleEncoder;

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);
    private final Rotation2d angleOffset;

    /** Instantiating motors and Encoders */
    public SwerveModuleReal(int driveMotorID, int angleMotorID, int cancoderID,
        Rotation2d cancoderOffset) {

        this.angleOffset = cancoderOffset;

        angleEncoder = new CANcoder(cancoderID, "canivore");
        mDriveMotor = new TalonFX(driveMotorID, "canivore");
        mAngleMotor = new TalonFX(angleMotorID, "canivore");

        configAngleEncoder();
        configAngleMotor();
        configDriveMotor();

        driveMotorSelectedPosition = mDriveMotor.getPosition();
        driveMotorSelectedSensorVelocity = mDriveMotor.getVelocity();
        angleMotorSelectedPosition = mAngleMotor.getPosition();
        absolutePositionAngleEncoder = angleEncoder.getAbsolutePosition();
    }

    private void configAngleMotor() {
        /* Angle Motor Config */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        swerveAngleFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = 1.0;
        swerveAngleFXConfig.Feedback.RotorToSensorRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit =
            Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime =
            Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        mAngleMotor.getConfigurator().apply(swerveAngleFXConfig);
    }

    private void configDriveMotor() {
        /* Drive Motor Config */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
            Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit =
            Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime =
            Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kS = Constants.Swerve.driveKS;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.driveKV;
        swerveDriveFXConfig.Slot0.kA = Constants.Swerve.driveKA;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
            Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
            Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
            Constants.Swerve.closedLoopRamp;

        mDriveMotor.getConfigurator().apply(swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);
    }

    private void configAngleEncoder() {
        /* Angle Encoder Config */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        swerveCANcoderConfig.MagnetSensor.MagnetOffset = -angleOffset.getRotations();

        angleEncoder.getConfigurator().apply(swerveCANcoderConfig);
    }

    @Override
    public void setAngleMotor(double angle) {
        mAngleMotor.setControl(anglePosition.withPosition(angle));
    }

    @Override
    public void setDriveMotor(double mps) {
        // driveVelocity.FeedForward = feedforward;
        double driveRPS = mps / Constants.Swerve.wheelCircumference.in(Meter);
        driveVelocity.Velocity = driveRPS;
        mDriveMotor.setControl(driveVelocity);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        BaseStatusSignal.refreshAll(driveMotorSelectedPosition, driveMotorSelectedSensorVelocity,
            angleMotorSelectedPosition, absolutePositionAngleEncoder);
        inputs.driveMotorSelectedPosition = driveMotorSelectedPosition.getValue();
        inputs.driveMotorSelectedSensorVelocity = driveMotorSelectedSensorVelocity.getValue();
        inputs.angleMotorSelectedPosition = angleMotorSelectedPosition.getValue();
        inputs.absolutePositionAngleEncoder = absolutePositionAngleEncoder.getValue();
        // inputs.driveMotorTemp = mDriveMotor.getDeviceTemp().getValueAsDouble();
        // inputs.angleMotorTemp = mAngleMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setPositionAngleMotor(double absolutePosition) {
        mAngleMotor.setPosition(absolutePosition);
    }

}
