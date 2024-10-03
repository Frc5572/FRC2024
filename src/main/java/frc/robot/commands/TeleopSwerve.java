package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Creates an command for driving the swerve drive during tele-op
 */
public class TeleopSwerve extends Command {

    private boolean fieldRelative;
    private boolean openLoop;
    private Swerve swerveDrive;
    private CommandXboxController controller;
    private double speedMultiplier = 1;

    /**
     * Creates a command for driving the swerve drive during tele-op
     *
     * @param swerveDrive The instance of the swerve drive subsystem
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param openLoop Open or closed loop system
     */
    public TeleopSwerve(Swerve swerveDrive, CommandXboxController controller, boolean fieldRelative,
        boolean openLoop) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.controller = controller;
    }

    /**
     * Creates a command for driving the swerve drive during tele-op
     *
     * @param swerveDrive The instance of the swerve drive subsystem
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param openLoop Open or closed loop system
     * @param speedMultiplier Speed multiplier to increase or decrease speed
     */
    public TeleopSwerve(Swerve swerveDrive, CommandXboxController controller, boolean fieldRelative,
        boolean openLoop, double speedMultiplier) {
        this(swerveDrive, controller, fieldRelative, openLoop);
        this.speedMultiplier = speedMultiplier;
    }

    @Override
    public void execute() {
        Robot.profiler.push("teleop_swerve");
        double yaxis = -controller.getLeftY() * speedMultiplier;
        double xaxis = -controller.getLeftX() * speedMultiplier;
        double raxis = -controller.getRightX() * speedMultiplier;

        /* Deadbands */
        yaxis = (Math.abs(yaxis) < Constants.STICK_DEADBAND) ? 0
            : (yaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
        xaxis = (Math.abs(xaxis) < Constants.STICK_DEADBAND) ? 0
            : (xaxis - Constants.STICK_DEADBAND) / (1.0 - Constants.STICK_DEADBAND);
        xaxis *= xaxis * Math.signum(xaxis);
        yaxis *= yaxis * Math.signum(yaxis);
        raxis = (Math.abs(raxis) < Constants.STICK_DEADBAND) ? 0 : raxis;
        // System.out.println(swerveDrive.getStringYaw());

        Translation2d translation =
            new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
        double rotation = raxis * Constants.Swerve.maxAngularVelocity;
        swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
        Robot.profiler.pop();
    }
}
