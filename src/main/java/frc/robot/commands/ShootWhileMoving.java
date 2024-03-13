package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Shooting while moving command
 */
public class ShootWhileMoving extends Command {

    private Swerve swerveDrive;
    private CommandXboxController controller;
    private Supplier<Pose2d> swervePoseSupplier;
    private Supplier<Translation2d> targetPositionSupplier;

    private PIDController pidController = new PIDController(Constants.Swerve.AUTO_ROTATION_KP,
        Constants.Swerve.AUTO_ROTATION_KI, Constants.Swerve.AUTO_ROTATION_KD);

    /**
     * Shoot while moving command
     *
     * @param swerveDrive Swerve Drive subsystem
     * @param controller Driver Controller
     */
    public ShootWhileMoving(Swerve swerveDrive, CommandXboxController controller,
        Supplier<Pose2d> swervePoseSupplier, Supplier<Translation2d> targetPositionSupplier) {
        this.swervePoseSupplier = swervePoseSupplier;
        this.targetPositionSupplier = targetPositionSupplier;
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.controller = controller;
        // We use radians here because PathPlanner uses radians internally. That way we can reuse
        // PID gains from autonomous.
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(Math.toRadians(.5));
    }

    @Override
    public void execute() {
        double yaxis = -controller.getLeftY();
        double xaxis = -controller.getLeftX();

        /* Deadbands */
        yaxis = (Math.abs(yaxis) < Constants.STICK_DEADBAND) ? 0
            : (yaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
        xaxis = (Math.abs(xaxis) < Constants.STICK_DEADBAND) ? 0
            : (xaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
        xaxis *= xaxis * Math.signum(xaxis);
        yaxis *= yaxis * Math.signum(yaxis);
        // System.out.println(swerveDrive.getStringYaw());

        Translation2d translation =
            new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);

        Pose2d futurePose = swervePoseSupplier.get().plus(
            new Transform2d(translation.times(Constants.LEAD_GAIN), Rotation2d.fromRotations(0)));

        Rotation2d desiredRotation = getDesiredRotation(futurePose, targetPositionSupplier.get());

        SmartDashboard.putNumber("Move Shoot Desired Rotation", desiredRotation.getDegrees());
        pidController.setSetpoint(desiredRotation.getRadians());
        double rotation =
            pidController.calculate(swervePoseSupplier.get().getRotation().getRadians());
        if (pidController.atSetpoint()) {
            rotation = 0;
        }
        RobotContainer.readyShoot.setBoolean(pidController.atSetpoint());
        swerveDrive.drive(translation, rotation, true, false);
    }

    /**
     * Get target rotation for a given pose to shoot into the speaker.
     */
    public static Rotation2d getDesiredRotation(Pose2d swervePose, Translation2d target) {
        Rotation2d desiredRotation = target.minus(swervePose.getTranslation()).getAngle();
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            desiredRotation = desiredRotation.plus(Rotation2d.fromDegrees(5));
        } else {
            desiredRotation = desiredRotation.plus(Rotation2d.fromDegrees(5));
        }
        return desiredRotation;
    }

}
