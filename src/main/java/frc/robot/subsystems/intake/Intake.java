package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * Intake Subsystem
 */
public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeInputsAutoLogged intakeAutoLogged = new IntakeInputsAutoLogged();

    private GenericEntry beamBrake = RobotContainer.mainDriverTab.add("Have Note", false)
        .withWidget(BuiltInWidgets.kBooleanBox).withPosition(9, 4).withSize(3, 2).getEntry();

    public Intake(IntakeIO io) {
        this.io = io;
        io.updateInputs(intakeAutoLogged);
    }

    @Override
    public void periodic() {
        io.updateInputs(intakeAutoLogged);
        Logger.processInputs("Intake", intakeAutoLogged);
        beamBrake.setBoolean(!getSensorStatus());
    }

    public void setIntakeMotor(double percentage) {
        Logger.recordOutput("/Intake/Intake Percentage", percentage);
        io.setIntakeMotorPercentage(percentage);
    }

    public void setIndexerMotor(double percentage) {
        Logger.recordOutput("/Intake/Indexer Percentage", percentage);
        io.setIndexerMotorPercentage(percentage);
    }

    public boolean getSensorStatus() {
        return intakeAutoLogged.sensorStatus;
    }

    /**
     * Command to run the intake motor and indexer until the sensor trips
     *
     * @return {@link Command} to run the intake and indexer motors
     */
    public Command runIntakeMotor(double intakeSpeed, double indexerSpeed) {
        return Commands.startEnd(() -> {
            setIntakeMotor(intakeSpeed);
            setIndexerMotor(indexerSpeed);
        }, () -> {
            setIntakeMotor(0);
            setIndexerMotor(0);
        }, this).until(() -> !getSensorStatus()).unless(() -> !getSensorStatus());
    }

    /**
     * Command to run the intake motor and indexer until the sensor trips
     *
     * @return {@link Command} to run the intake and indexer motors
     */
    public Command runIntakeMotorNonStop(double intakeSpeed, double indexerSpeed) {
        return Commands.startEnd(() -> {
            setIntakeMotor(intakeSpeed);
            setIndexerMotor(indexerSpeed);
        }, () -> {
            setIntakeMotor(0);
            setIndexerMotor(0);
        }, this);
    }

    /**
     * Command to run the indexer
     *
     * @return {@link Command} to run the indexer motors
     */
    public Command runIndexerMotor(double speed) {
        return Commands.startEnd(() -> {
            setIndexerMotor(speed);
        }, () -> {
            setIndexerMotor(0);
        }, this);
    }
}
