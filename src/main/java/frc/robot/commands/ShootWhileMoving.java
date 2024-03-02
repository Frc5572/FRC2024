package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

public class ShootWhileMoving extends Command {

    private Swerve swerveDrive;
    private CommandXboxController controller;

    private PIDController pidController = new PIDController(Constants.Swerve.AUTO_ROTATION_KP,
        Constants.Swerve.AUTO_ROTATION_KI, Constants.Swerve.AUTO_ROTATION_KD);

    public ShootWhileMoving(Swerve swerveDrive, CommandXboxController controller) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.controller = controller;
        pidController.enableContinuousInput(-Math.PI, Math.PI); // We use radians here because
                                                                // PathPlanner uses radians
                                                                // internally. That way we can reuse
                                                                // PID gains from autonomous.
        pidController.setTolerance(Math.toRadians(.5));
    }

    @Override
    public void execute() {
        double yaxis = -controller.getLeftY();
        double xaxis = -controller.getLeftX();

        /* Deadbands */
        yaxis = (Math.abs(yaxis) < Constants.STICK_DEADBAND) ? 0 : yaxis;
        xaxis = (Math.abs(xaxis) < Constants.STICK_DEADBAND) ? 0 : xaxis;
        // System.out.println(swerveDrive.getStringYaw());

        Translation2d translation =
            new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);

        Pose2d futurePose = swerveDrive.getPose().plus(
            new Transform2d(translation.times(Constants.LEAD_GAIN), Rotation2d.fromRotations(0)));

        Rotation2d desiredRotation = FieldConstants.Speaker.centerSpeakerOpening.getTranslation()
            .minus(futurePose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(5));

        SmartDashboard.putNumber("Move Shoot Desired Rotation", desiredRotation.getDegrees());
        pidController.setSetpoint(desiredRotation.getRadians());
        double rotation = pidController.calculate(swerveDrive.getPose().getRotation().getRadians());
        if (pidController.atSetpoint()) {
            rotation = 0;
        }
        swerveDrive.drive(translation, rotation, true, false);
    }

}
