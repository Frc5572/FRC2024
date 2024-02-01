package frc.robot;

import java.util.List;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterVortex;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveReal;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);

    // Initialize AutoChooser Sendable
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private Swerve s_Swerve;
    private Shooter shooter;
    private Intake intake;
    private ElevatorWrist elevatorWrist;

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Wait 1 Second", "wait");
        switch (runtimeType) {
            case kReal:
                s_Swerve = new Swerve(new SwerveReal());
                shooter = new Shooter(new ShooterVortex());
                intake = new Intake(new IntakeIOFalcon());
                break;
            case kSimulation:
                // s_Swerve = new Swerve(new SwerveIO() {});
                break;
            default:
                s_Swerve = new Swerve(new SwerveIO() {});
                shooter = new Shooter(new ShooterIO() {});
                intake = new Intake(new IntakeIO() {});
        }
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver,
            Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop));
        // autoChooser.addOption(resnickAuto, new ResnickAuto(s_Swerve));
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() { /* Driver Buttons */
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        Command autocommand;
        NamedCommands.registerCommand("Intake Command",
            CommandFactory.runIntake(intake, elevatorWrist));
        // NamedComands.registerCommand("Shoot Command", shooterCommand when done )
        String stuff = autoChooser.getSelected();
        switch (stuff) {
            case "P1 2Ns Scoring Close":
                List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("1st Auto");
                Pose2d initialState = paths.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState))
                    .andThen(new PathPlannerAuto("1st Auto"));

                break;
            case "P1 3Ns Scoring":
                List<PathPlannerPath> paths2 = PathPlannerAuto.getPathGroupFromAutoFile("2nd Auto");
                Pose2d initialState2 = paths2.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState2);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState2))
                    .andThen(new PathPlannerAuto("2nd Auto"));

                break;
            case "P2 3Ns Scoring":
                List<PathPlannerPath> paths3 = PathPlannerAuto.getPathGroupFromAutoFile("3rd Auto");
                Pose2d initialState3 = paths3.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState3);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState3))
                    .andThen(new PathPlannerAuto("3rd Auto"));

                break;
            case "P1 4Ns Scoring Close":
                List<PathPlannerPath> paths4 = PathPlannerAuto.getPathGroupFromAutoFile("4th Auto");
                Pose2d initialState4 = paths4.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState4);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState4))
                    .andThen(new PathPlannerAuto("4th Auto"));

                break;
            case "P1 4Ns Scoring far":
                List<PathPlannerPath> paths5 = PathPlannerAuto.getPathGroupFromAutoFile("5th Auto");
                Pose2d initialState5 = paths5.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState5);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState5))
                    .andThen(new PathPlannerAuto("5th Auto"));

                break;
            case "P1 5Ns Scoring":
                List<PathPlannerPath> paths6 = PathPlannerAuto.getPathGroupFromAutoFile("6th Auto");
                Pose2d initialState6 = paths6.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState6);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState6))
                    .andThen(new PathPlannerAuto("6th Auto"));

                break;
            case "P2 5Ns Scoring":
                List<PathPlannerPath> paths7 = PathPlannerAuto.getPathGroupFromAutoFile("7th Auto");
                Pose2d initialState7 = paths7.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState7);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState7))
                    .andThen(new PathPlannerAuto("7th Auto"));

                break;
            case "P1 6Ns Scoring":
                List<PathPlannerPath> paths8 = PathPlannerAuto.getPathGroupFromAutoFile("8th Auto");
                Pose2d initialState8 = paths8.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState8);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState8))
                    .andThen(new PathPlannerAuto("8th Auto"));

                break;
            case "P3 3Ns Scoring":
                List<PathPlannerPath> paths9 = PathPlannerAuto.getPathGroupFromAutoFile("9th Auto");
                Pose2d initialState9 = paths9.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState9);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState9))
                    .andThen(new PathPlannerAuto("9th Auto"));

                break;
            case "P3 N1 and steal 2N":
                List<PathPlannerPath> paths10 =
                    PathPlannerAuto.getPathGroupFromAutoFile("10th Auto");
                Pose2d initialState10 = paths10.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState10);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState10))
                    .andThen(new PathPlannerAuto("10th Auto"));

                break;
            case "P3 N1 and Steals 3N":
                List<PathPlannerPath> paths11 =
                    PathPlannerAuto.getPathGroupFromAutoFile("Scores Steals Notes Auto");
                Pose2d initialState11 = paths11.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState11);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState11))
                    .andThen(new PathPlannerAuto("Scores Steals Notes Auto"));

                break;
            case "P1 N2s Scoring far":
                List<PathPlannerPath> paths12 =
                    PathPlannerAuto.getPathGroupFromAutoFile("Scoring N7 and N8");
                Pose2d initialState12 = paths12.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState12);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState12))
                    .andThen(new PathPlannerAuto("Scoring N7 and N8"));

                break;
            default:
                autocommand = new WaitCommand(1.0);
        }
        return autocommand;
    }
}
