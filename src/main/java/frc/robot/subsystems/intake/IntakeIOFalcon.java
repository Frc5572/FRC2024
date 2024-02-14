package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Intake IO Layer with real motors and sensors
 */
public class IntakeIOFalcon implements IntakeIO {

    // private final TalonFX intakeMotor =
    // new TalonFX(Constants.Motors.Intake.INTAKE_MOTOR_ID, "canivore");
    private final CANSparkMax intakeMotor =
        new CANSparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final TalonFX indexerMotor = new TalonFX(Constants.Motors.Intake.INDEXER_MOTOR_ID);

    private final DutyCycleOut intakeDutyCycleOut = new DutyCycleOut(0);
    private final DutyCycleOut indexerDutyCycleOut = new DutyCycleOut(0);
    private final DigitalInput beamBrake = new DigitalInput(8);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOFalcon() {
        intakeMotor.setInverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        indexerMotor.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeSupplyVoltage = intakeMotor.getBusVoltage();
        // inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakeAmps = intakeMotor.getOutputCurrent();
        inputs.intakeRPM = intakeMotor.getEncoder().getVelocity();
        inputs.indexerSupplyVoltage = indexerMotor.getSupplyVoltage().getValueAsDouble();
        inputs.indexerMotorVoltage = indexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.indexerAmps = indexerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.indexerRPM = indexerMotor.getVelocity().getValueAsDouble();
        inputs.sensorStatus = beamBrake.get(); // true == no game piece
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        // intakeMotor.setControl(intakeDutyCycleOut.withOutput(percent));
        intakeMotor.set(percent);
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        indexerMotor.setControl(indexerDutyCycleOut.withOutput(percent));
    }
}
