package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeInputsAutoLogged intakeAutoLogged = new IntakeInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
        io.updateInputs(intakeAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(intakeAutoLogged);
        Logger.processInputs("Intake", intakeAutoLogged);
    }

    public void setIntakeMotor(double percentage) {
        io.setIntakeMotorPercentage(percentage);
    }

    public void setIndexerMotor(double percentage) {
        io.setIndexerMotorPercentage(percentage);
    }

    public boolean getSensorStatus() {
        return intakeAutoLogged.sensorStatus;
    }

    public Command runIntakeMotor() {
        return Commands.startEnd(() -> {
            setIntakeMotor(0.5);
            setIndexerMotor(0.5);
        }, () -> {
            setIntakeMotor(0);
            setIndexerMotor(0);
        }, this).until(() -> getSensorStatus()).unless(() -> getSensorStatus());
    }
}
