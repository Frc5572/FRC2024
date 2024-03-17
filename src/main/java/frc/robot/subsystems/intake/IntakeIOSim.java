package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/**
 * Intake IO Layer for simulation
 */
public class IntakeIOSim implements IntakeIO {

    private final FlywheelSim intakeMotorLeft = new FlywheelSim(DCMotor.getNEO(1), 5, 1);
    private final FlywheelSim intakeMotorRight = new FlywheelSim(DCMotor.getNEO(1), 5, 1);
    private final FlywheelSim indexerMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 1);

    private final DigitalInput indexerBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INDEXER_BEAM_BRAKE_DIO_PORT);
    private final DigitalInput intakeBeamBrake =
        new DigitalInput(Constants.IntakeConstants.INTAKE_BEAM_BRAKE_DIO_PORT);

    /**
     * Intake IO Layer with real motors and sensors
     */
    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.indexerBeamBrake = !indexerBeamBrake.get(); // true == game piece
        inputs.intakeBeamBrake = !intakeBeamBrake.get(); // true == game piece
    }

    @Override
    public void setIntakeMotorPercentage(double percent) {
        intakeMotorLeft.setInputVoltage(percent * RobotController.getBatteryVoltage());
        intakeMotorRight.setInputVoltage(percent * RobotController.getBatteryVoltage());
    }

    @Override
    public void setIndexerMotorPercentage(double percent) {
        indexerMotor.setInputVoltage(percent * RobotController.getBatteryVoltage());
    }
}
