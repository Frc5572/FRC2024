package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Intake IO Layer with real motors and sensors
 */
public class IntakeIOFalcon implements IntakeIO {

    private final CANSparkMax intakeMotorLeft =
        new CANSparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID_LEFT, MotorType.kBrushless);
    private final CANSparkMax intakeMotorRight =
        new CANSparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID_RIGHT, MotorType.kBrushless);
    public final RelativeEncoder intakeRelativeEnc =
        intakeMotorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    private final TalonFX indexerMotor = new TalonFX(Constants.Motors.Intake.INDEXER_MOTOR_ID);

    private final DutyCycleOut indexerDutyCycleOut = new DutyCycleOut(0);
    private final DigitalInput beamBrake = new DigitalInput(8);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOFalcon() {
        intakeMotorLeft.setInverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED);
        intakeMotorLeft.setIdleMode(IdleMode.kCoast);
        intakeMotorRight.restoreFactoryDefaults();
        intakeMotorRight.setInverted(false);
        indexerMotor.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeSupplyVoltage = intakeMotorLeft.getBusVoltage();
        inputs.intakeAmps = intakeMotorLeft.getOutputCurrent();
        inputs.intakeRPM = intakeRelativeEnc.getVelocity();
        // inputs.intakeTemp = intakeMotorLeft.getMotorTemperature();
        inputs.indexerSupplyVoltage = indexerMotor.getSupplyVoltage().getValueAsDouble();
        inputs.indexerMotorVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.indexerAmps = indexerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.indexerRPM = indexerMotor.getVelocity().getValueAsDouble();
        // inputs.indexerTemp = indexerMotor.getDeviceTemp().getValueAsDouble();
        inputs.sensorStatus = beamBrake.get(); // true == no game piece
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        intakeMotorLeft.set(percent);
        intakeMotorRight.set(percent);
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        indexerMotor.setControl(indexerDutyCycleOut.withOutput(percent));
    }
}
