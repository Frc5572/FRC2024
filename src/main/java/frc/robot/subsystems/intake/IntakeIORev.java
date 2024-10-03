package frc.robot.subsystems.intake;

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
public class IntakeIORev implements IntakeIO {

    private final CANSparkMax intakeMotorLeft =
        new CANSparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID_LEFT, MotorType.kBrushless);
    private final CANSparkMax intakeMotorRight =
        new CANSparkMax(Constants.Motors.Intake.INTAKE_MOTOR_ID_RIGHT, MotorType.kBrushless);
    public final RelativeEncoder intakeRelativeEnc =
        intakeMotorLeft.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    private final DigitalInput intakeBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INTAKE_BEAM_BRAKE_DIO_PORT);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIORev() {
        intakeMotorLeft.restoreFactoryDefaults();
        intakeMotorRight.restoreFactoryDefaults();
        intakeMotorLeft.setInverted(Constants.IntakeConstants.INTAKE_MOTOR_INVERTED);
        intakeMotorRight.setInverted(false);
        intakeMotorLeft.setIdleMode(IdleMode.kCoast);
        intakeMotorRight.setIdleMode(IdleMode.kCoast);
        intakeMotorLeft.setSmartCurrentLimit(40);
        intakeMotorRight.setSmartCurrentLimit(40);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeBeamBrake = !intakeBeamBrake.get(); // true == game piece
        inputs.intakeRPM = intakeRelativeEnc.getVelocity();
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        // Left ratio is 60:30
        // Right ratio is 32:30
        intakeMotorLeft.set(percent);
        intakeMotorRight.set(percent);
    }
}
