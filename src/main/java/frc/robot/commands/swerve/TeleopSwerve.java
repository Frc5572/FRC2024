package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

/**
 * Creates an command for driving the swerve drive during tele-op
 */
public class TeleopSwerve extends Command {

    private boolean fieldRelative;
    private boolean openLoop;
    private Swerve swerveDrive;
    private CommandXboxController controller;

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

    @Override
    public void execute() {
        double yaxis = -controller.getLeftY();
        double xaxis = -controller.getLeftX();
        double raxis = -controller.getRightX();

        /* Deadbands */
        yaxis = (Math.abs(yaxis) < Constants.STICK_DEADBAND) ? 0 : yaxis;
        xaxis = (Math.abs(xaxis) < Constants.STICK_DEADBAND) ? 0 : xaxis;
        raxis = (Math.abs(raxis) < Constants.STICK_DEADBAND) ? 0 : raxis;
        // System.out.println(swerveDrive.getStringYaw());

        Translation2d translation =
            new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
        double rotation = raxis * Constants.Swerve.maxAngularVelocity;
        swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
    }
}
