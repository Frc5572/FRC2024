package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Move to Position
 */
public class MoveToPos extends Command {

    public Swerve swerve;
    public Pose2d pose2d;
    public Supplier<Pose2d> pose2dSupplier;
    private boolean flipForRed = true;

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(Constants.SwerveTransformPID.PID_XKP,
            Constants.SwerveTransformPID.PID_XKI, Constants.SwerveTransformPID.PID_XKD),
        new PIDController(Constants.SwerveTransformPID.PID_YKP,
            Constants.SwerveTransformPID.PID_YKI, Constants.SwerveTransformPID.PID_YKD),
        new ProfiledPIDController(Constants.SwerveTransformPID.PID_TKP,
            Constants.SwerveTransformPID.PID_TKI, Constants.SwerveTransformPID.PID_TKD,
            new TrapezoidProfile.Constraints(Constants.SwerveTransformPID.MAX_ANGULAR_VELOCITY,
                Constants.SwerveTransformPID.MAX_ANGULAR_ACCELERATION)));

    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     * @param pose2dSupplier Supplier of Pose2d
     * @param flipForRed Flip the Pose2d relative to the Red Alliance
     */
    public MoveToPos(Swerve swerve, Supplier<Pose2d> pose2dSupplier, boolean flipForRed,
        double tol) {
        this(swerve, pose2dSupplier, flipForRed, tol, Constants.SwerveTransformPID.PID_TKP);
    }

    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     * @param pose2dSupplier Supplier of Pose2d
     * @param flipForRed Flip the Pose2d relative to the Red Alliance
     */
    public MoveToPos(Swerve swerve, Supplier<Pose2d> pose2dSupplier, boolean flipForRed, double tol,
        double turnP) {
        this.swerve = swerve;
        this.pose2dSupplier = pose2dSupplier;
        this.flipForRed = flipForRed;
        this.addRequirements(swerve);
        holonomicDriveController.getThetaController().setP(turnP);
        holonomicDriveController.setTolerance(new Pose2d(tol, tol, Rotation2d.fromDegrees(1)));
    }

    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     * @param pose2dSupplier Supplier of Pose2d
     * @param flipForRed Flip the Pose2d relative to the Red Alliance
     */
    public MoveToPos(Swerve swerve, Supplier<Pose2d> pose2dSupplier, boolean flipForRed) {
        this(swerve, pose2dSupplier, flipForRed, 0.05);
    }


    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     * @param pose2dSupplier Supplier of Pose2d
     */
    public MoveToPos(Swerve swerve, Supplier<Pose2d> pose2dSupplier) {
        this(swerve, pose2dSupplier, true);
    }

    /**
     * Move to Position
     *
     * @param swerve Swerve Drive Subsystem
     */
    public MoveToPos(Swerve swerve) {
        this(swerve, () -> new Pose2d());
    }

    @Override
    public void initialize() {
        pose2d = pose2dSupplier.get();
        if (flipForRed && DriverStation.getAlliance().get() == Alliance.Red) {
            pose2d = FieldConstants.allianceFlip(pose2d);
        }
    }

    @Override
    public void execute() {
        ChassisSpeeds ctrlEffort =
            holonomicDriveController.calculate(swerve.getPose(), pose2d, 0, pose2d.getRotation());
        swerve.setModuleStates(ctrlEffort);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setMotorsZero();
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}
