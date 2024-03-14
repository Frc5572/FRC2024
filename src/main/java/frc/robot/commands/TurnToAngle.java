package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * This command will turn the robot to a specified angle.
 */
public class TurnToAngle extends Command {

    private Swerve swerve;
    private HolonomicDriveController holonomicDriveController =
        new HolonomicDriveController(new PIDController(0, 0, 0), new PIDController(0, 0, 0),
            new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP / 2,
                Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
                new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                    Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));
    private Rotation2d targetRotation2d;
    private Pose2d position;
    private Rotation2d offset = new Rotation2d();

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve Swerve subsystem
     * @param angle Requested angle to turn to
     */
    public TurnToAngle(Swerve swerve, Rotation2d angle) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.offset = angle;
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(1)));
    }

    @Override
    public void initialize() {
        this.targetRotation2d = offset.plus(Rotation2d.fromDegrees(180));
        if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            this.targetRotation2d = Rotation2d.fromDegrees(180).minus(offset);
        }
    }

    @Override
    public void execute() {
        position = new Pose2d(0, 0, swerve.getFieldRelativeHeading());
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(position,
            new Pose2d(0, 0, targetRotation2d), 0, targetRotation2d);
        swerve.setModuleStates(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupt) {
        swerve.setMotorsZero();
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}
