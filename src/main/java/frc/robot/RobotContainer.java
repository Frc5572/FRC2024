package frc.robot;

import java.util.List;
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
import frc.lib.util.photon.PhotonCameraWrapper;
import frc.lib.util.photon.PhotonReal;
import frc.robot.Robot.RobotRunType;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberNEO;
import frc.robot.subsystems.elevatorWrist.ElevatorWrist;
import frc.robot.subsystems.elevatorWrist.ElevatorWristIO;
import frc.robot.subsystems.elevatorWrist.ElevatorWristReal;
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
    public final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);
    private final CommandXboxController test = new CommandXboxController(4);


    // Initialize AutoChooser Sendable
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private Swerve s_Swerve;
    private Shooter shooter;
    private Intake intake;
    private PhotonCameraWrapper[] cameras;
    public ElevatorWrist elevatorWrist;
    public Climber climber;

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

        cameras =
            /*
             * Camera Order: 0 - Front Left 1 - Front RIght 2 - Back Left 3 - Back Right
             */
            new PhotonCameraWrapper[] {
                new PhotonCameraWrapper(
                    new PhotonReal(Constants.CameraConstants.FrontLeftFacingCamera.CAMERA_NAME),
                    Constants.CameraConstants.FrontLeftFacingCamera.KCAMERA_TO_ROBOT),
                new PhotonCameraWrapper(
                    new PhotonReal(Constants.CameraConstants.FrontRightFacingCamera.CAMERA_NAME),
                    Constants.CameraConstants.FrontRightFacingCamera.KCAMERA_TO_ROBOT),
                new PhotonCameraWrapper(
                    new PhotonReal(Constants.CameraConstants.BackLeftFacingCamera.CAMERA_NAME),
                    Constants.CameraConstants.BackLeftFacingCamera.KCAMERA_TO_ROBOT),
                new PhotonCameraWrapper(
                    new PhotonReal(Constants.CameraConstants.BackRightFacingCamera.CAMERA_NAME),
                    Constants.CameraConstants.BackRightFacingCamera.KCAMERA_TO_ROBOT)};

        switch (runtimeType) {
            case kReal:
                shooter = new Shooter(new ShooterVortex());
                intake = new Intake(new IntakeIOFalcon());
                s_Swerve = new Swerve(new SwerveReal(), cameras);
                elevatorWrist = new ElevatorWrist(new ElevatorWristReal(), operator);
                climber = new Climber(new ClimberNEO());
                break;
            case kSimulation:
                s_Swerve = new Swerve(new SwerveIO() {}, cameras);
                shooter = new Shooter(new ShooterIO() {});
                intake = new Intake(new IntakeIO() {});
                elevatorWrist = new ElevatorWrist(new ElevatorWristIO() {}, operator);
                climber = new Climber(new ClimberIO() {});
                break;
            default:
                s_Swerve = new Swerve(new SwerveIO() {}, cameras);
                shooter = new Shooter(new ShooterIO() {});
                intake = new Intake(new IntakeIO() {});
                elevatorWrist = new ElevatorWrist(new ElevatorWristIO() {}, operator);
                climber = new Climber(new ClimberIO() {});
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
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));
        // intake forward
        driver.a().whileTrue(intake.runIntakeMotor(1, .20));
        // intake backward
        driver.b().whileTrue(intake.runIndexerMotor(-.1));
        driver.x().whileTrue(CommandFactory.passThroughShoot(shooter, intake));

        operator.x().whileTrue(CommandFactory.spit(shooter, intake));
        operator.rightBumper().whileTrue(CommandFactory.shootSpeaker(shooter, intake));
        operator.start().onTrue(elevatorWrist.ampPosition());
        operator.back().onTrue(elevatorWrist.homePosition());

        // operator.povDown().whileTrue(
        // elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.AMP_HEIGHT,
        // Constants.ElevatorWristConstants.SetPoints.AMP_ANGLE));
        // operator.povUp().whileTrue(
        // elevatorWrist.goToPosition(Constants.ElevatorWristConstants.SetPoints.TRAP_HEIGHT,
        // Constants.ElevatorWristConstants.SetPoints.TRAP_ANGLE));

        // // climber forward
        // operator.a().whileTrue(new StartEndCommand(() -> {
        // climber.setLeftPower(SmartDashboard.getNumber("Left Climber Power", 0));
        // climber.setRightPower(SmartDashboard.getNumber("Right Climber Power", 0));
        // }, () -> {
        // climber.setLeftPower(0);
        // climber.setRightPower(0);
        // }, climber));
        // // // climber backward
        // operator.b().whileTrue(new StartEndCommand(() -> {
        // climber.setLeftPower(-SmartDashboard.getNumber("Left Climber Power", 0));
        // climber.setRightPower(-SmartDashboard.getNumber("Right Climber Power", 0));
        // }, () -> {
        // climber.setLeftPower(0);
        // climber.setRightPower(0);
        // }, climber));
        // // // climber left
        // operator.x().whileTrue(new StartEndCommand(() -> {
        // climber.setLeftPower(-SmartDashboard.getNumber("Left Climber Power", 0));
        // }, () -> {
        // climber.setLeftPower(0);
        // climber.setRightPower(0);
        // }, climber));
        // // // climber right
        // operator.y().whileTrue(new StartEndCommand(() -> {
        // climber.setRightPower(-SmartDashboard.getNumber("Right Climber Power", 0));
        // }, () -> {
        // climber.setLeftPower(0);
        // climber.setRightPower(0);
        // }, climber));

        // // elevator forward
        // driver.leftBumper().whileTrue(new StartEndCommand(() -> {
        // elevatorWrist.setElevatorPower(-0.2);
        // }, () -> {
        // elevatorWrist.setElevatorPower(0.0);
        // }));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        Command autocommand;
        String stuff = autoChooser.getSelected();
        switch (stuff) {
            case "Test Auto":
                List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("New Auto");
                Pose2d initialState = paths.get(0).getPreviewStartingHolonomicPose();
                s_Swerve.resetOdometry(initialState);
                autocommand = new InstantCommand(() -> s_Swerve.resetOdometry(initialState))
                    .andThen(new PathPlannerAuto("New Auto"));

                break;
            default:
                autocommand = new WaitCommand(1.0);
        }
        return autocommand;
    }

}
