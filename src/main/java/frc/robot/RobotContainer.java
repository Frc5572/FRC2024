package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import frc.robot.subsystems.intake.IntakeIOFalcon;
import frc.robot.subsystems.shooter.Shooter;
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
    // public Climber climber;

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Wait 1 Second", "wait");
        SmartDashboard.putNumber("Intake Power", 0);
        SmartDashboard.putNumber("Left Climber Power", 0);
        SmartDashboard.putNumber("Right Climber Power", 0);
        SmartDashboard.putNumber("Elevator Power", 0);
        SmartDashboard.putNumber("Wrist Power", 0);

        switch (runtimeType) {
            case kReal:
                // s_Swerve = new Swerve(new SwerveReal());
                shooter = new Shooter(new ShooterVortex());
                intake = new Intake(new IntakeIOFalcon());
                // elevatorWrist = new ElevatorWrist(new ElevatorWristReal());
                // climber = new Climber(new ClimberNEO());
                break;
            case kSimulation:
                // s_Swerve = new Swerve(new SwerveIO() {});
                s_Swerve = new Swerve(new SwerveReal());
                break;
            default:
                s_Swerve = new Swerve(new SwerveIO() {});
                // shooter = new Shooter(new ShooterIO() {});
                // intake = new Intake(new IntakeIO() {});
                // elevatorWrist = new ElevatorWrist(new ElevatorWristIO() {});
                // climber = new Climber(new ClimberIO() {});
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
        // intake forward
        driver.a().whileTrue(intake.runIntakeMotor(1, .25));
        // intake backward
        driver.b().whileTrue(intake.runIntakeMotor(-1, -.25));

        driver.x().whileTrue(CommandFactory.shootSpeaker(shooter, intake));
        // climber forward
        // driver.start().whileTrue(new StartEndCommand(() -> {
        // climber.setLeftPower(SmartDashboard.getNumber("Left Climber Power", 0));
        // climber.setRightPower(SmartDashboard.getNumber("Right Climber Power", 0));
        // }, () -> {
        // climber.setLeftPower(0);
        // climber.setRightPower(0);
        // }, climber));
        // // climber backward
        // driver.back().whileTrue(new StartEndCommand(() -> {
        // climber.setLeftPower(-SmartDashboard.getNumber("Left Climber Power", 0));
        // climber.setRightPower(-SmartDashboard.getNumber("Right Climber Power", 0));
        // }, () -> {
        // climber.setLeftPower(0);
        // climber.setRightPower(0);
        // }, climber));

        // elevator forward
        // driver.start().whileTrue(new StartEndCommand(() -> {
        // elevatorWrist.setElevatorPower(SmartDashboard.getNumber("Elevator Power", 0));
        // }, () -> {
        // elevatorWrist.setElevatorPower(0.0);
        // }, climber));
        // // climber backward
        // driver.back().whileTrue(new StartEndCommand(() -> {
        // elevatorWrist.setElevatorPower(-SmartDashboard.getNumber("Elevator Power", 0));
        // }, () -> {
        // elevatorWrist.setElevatorPower(0.0);
        // }, climber));



    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        Command autocommand;
        Command runshooter = shooter.shootWithDistance(() -> s_Swerve.distanceFromSpeaker());
        Command runIndexer = intake.runIndexerMotor(Constants.IntakeConstants.INDEX_MOTOR_FORWARD);

        NamedCommands.registerCommand("Intake Command",
            CommandFactory.runIntake(intake, elevatorWrist));
        NamedCommands.registerCommand("Shoot Command",
            CommandFactory.shootSpeaker(shooter, intake).withTimeout(1));
        NamedCommands.registerCommand("Run Shooter", runshooter);
        NamedCommands.registerCommand("Run Indexer", runIndexer.withTimeout(1));


        String stuff = autoChooser.getSelected();
        switch (stuff) {
            case "P1 3Ns Scoring":
                autocommand = new PathPlannerAuto("1st Auto");
                break;

            case "P2/3 3Ns Scoring":
                autocommand = new PathPlannerAuto("2nd Auto");
                break;

            case "P1 4Ns Scoring Close":
                autocommand = new PathPlannerAuto("3th Auto");
                break;

            case "P1 4Ns Scoring far":
                autocommand = new PathPlannerAuto("4th Auto");
                break;

            case "P1 5Ns Scoring":
                autocommand = new PathPlannerAuto("6th Auto");
                break;

            case "P2 5Ns Scoring":
                autocommand = new PathPlannerAuto("7th Auto");
                break;

            case "P1 6Ns Scoring":
                autocommand = new PathPlannerAuto("8th Auto");
                break;

            case "P3 3Ns Scoring":
                autocommand = new PathPlannerAuto("9th Auto");
                break;

            default:
                autocommand = new WaitCommand(1.0);
        }
        return autocommand;
    }

}
