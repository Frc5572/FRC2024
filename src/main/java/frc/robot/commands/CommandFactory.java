package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;

/**
 * File to create commands using factories
 */
public class CommandFactory {

    public static Command runIntake(Intake intake, ElevatorWrist elevatorWrist) {
        BooleanSupplier sensor = () -> intake.getSensorStatus();
        Command moveElevatorWrist =
            elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE);
        Command runIntakeIndexer =
            intake.runIntakeMotor(Constants.IntakeConstants.INDEX_MOTOR_FORWARD);
        return moveElevatorWrist.andThen(runIntakeIndexer).unless(sensor);
    }
}
