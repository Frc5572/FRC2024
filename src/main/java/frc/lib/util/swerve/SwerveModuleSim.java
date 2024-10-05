package frc.lib.util.swerve;

import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class SwerveModuleSim implements SwerveModuleIO {
    public int moduleNumber;

    private double angle;
    private double distance;
    private double driveSpeed;

    public SwerveModuleSim() {

    }

    @Override
    public void setModNumber(int number) {
        this.moduleNumber = number;
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        this.distance += this.driveSpeed * LoggedRobot.defaultPeriodSecs;
        inputs.driveMotorSelectedPosition =
            Conversions.metersToRotations(this.distance, Constants.Swerve.wheelCircumference);
        inputs.driveMotorSelectedSensorVelocity = Conversions.metersPerSecondToRotationPerSecond(
            this.driveSpeed, Constants.Swerve.wheelCircumference);

        inputs.angleMotorSelectedPosition = angle;

        inputs.absolutePositionAngleEncoder = angle;
        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    }

    public void setDriveMotor(double mps) {
        this.driveSpeed = mps;
    }

    public void setAngleMotor(double angle) {
        this.angle = angle;
    }
}
