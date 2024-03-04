package frc.robot;

import java.util.List;
import java.util.Map;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.MatchCommand;
import frc.lib.util.photon.PhotonCameraWrapper;
import frc.lib.util.photon.PhotonReal;
import frc.robot.Robot.RobotRunType;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.ShootWhileMoving;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberNEO;
import frc.robot.subsystems.elevator_wrist.ElevatorWrist;
import frc.robot.subsystems.elevator_wrist.ElevatorWristIO;
import frc.robot.subsystems.elevator_wrist.ElevatorWristReal;
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
    /* Shuffleboard */
    public static ShuffleboardTab mainDriverTab = Shuffleboard.getTab("Main Driver");

    // Initialize AutoChooser Sendable
    private final SendableChooser<String> autoChooser = new SendableChooser<>();
    public ComplexWidget autoChooserWidget = mainDriverTab.add("Auto Chooser", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(12, 0).withSize(2, 1);
    public GenericEntry operatorState =
        mainDriverTab.add("Operator State", OperatorState.getCurrentState().displayName)
            .withWidget(BuiltInWidgets.kTextView).withPosition(8, 0).withSize(2, 1).getEntry();
    public GenericEntry operatorManualMode = RobotContainer.mainDriverTab.add("Manual Mode", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("true_color", 0xff00ffff, "false_color", 0xff770000))
        .withPosition(10, 0).withSize(1, 1).getEntry();
    public static GenericEntry readyShoot = RobotContainer.mainDriverTab
        .add("Ready To Shoot", false).withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("true_color", 0xff00ffff, "false_color", 0xff770000))
        .withPosition(11, 0).withSize(1, 1).getEntry();

    public static final SendableChooser<Integer> numNoteChooser = new SendableChooser<>();
    public ComplexWidget numNoteChooserrWidget =
        mainDriverTab.add("Number of Additional Auto Notes", numNoteChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(12, 1).withSize(2, 1);
    public SimpleWidget fmsInfo =
        RobotContainer.mainDriverTab.add("FMS Info", 0).withWidget("FMSInfo")
            .withProperties(Map.of("topic", "/FMSInfo")).withPosition(4, 4).withSize(3, 1);
    public GenericEntry matchTime = RobotContainer.mainDriverTab.add("Match Time", 0)
        .withWidget("Match Time").withProperties(Map.of("time_display_mode", "Minutes and Seconds"))
        .withPosition(1, 4).withSize(3, 1).getEntry();
    public SimpleWidget voltageInfo =
        RobotContainer.mainDriverTab.add("Battery Voltage", 0).withWidget("Voltage View")
            .withProperties(Map.of("topic", "/AdvantageKit/SystemStats/BatteryVoltage"))
            .withPosition(2, 5).withSize(4, 1);
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);
    // private final CommandXboxController test = new CommandXboxController(4);



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
        autoChooser.setDefaultOption("Wait 1 Second", "wait");
        SmartDashboard.putNumber("Intake Power", 0);
        SmartDashboard.putNumber("Left Climber Power", 0);
        SmartDashboard.putNumber("Right Climber Power", 0);
        SmartDashboard.putNumber("Elevator Power", 0);
        SmartDashboard.putNumber("Wrist Power", 0);
        numNoteChooser.setDefaultOption("0", 0);
        for (int i = 0; i < 7; i++) {
            numNoteChooser.addOption(String.valueOf(i), i);
        }

        cameras =
            /*
             * Camera Order: 0 - Front Left 1 - Front RIght 2 - Back Left 3 - Back Right
             */
            new PhotonCameraWrapper[] {
                // new PhotonCameraWrapper(
                // new PhotonReal(Constants.CameraConstants.FrontLeftFacingCamera.CAMERA_NAME),
                // Constants.CameraConstants.FrontLeftFacingCamera.KCAMERA_TO_ROBOT),
                new PhotonCameraWrapper(
                    new PhotonReal(Constants.CameraConstants.FrontRightFacingCamera.CAMERA_NAME),
                    Constants.CameraConstants.FrontRightFacingCamera.KCAMERA_TO_ROBOT)};
        // new PhotonCameraWrapper(
        // new PhotonReal(Constants.CameraConstants.BackLeftFacingCamera.CAMERA_NAME),
        // Constants.CameraConstants.BackLeftFacingCamera.KCAMERA_TO_ROBOT),
        // new PhotonCameraWrapper(
        // new PhotonReal(Constants.CameraConstants.BackRightFacingCamera.CAMERA_NAME),
        // Constants.CameraConstants.BackRightFacingCamera.KCAMERA_TO_ROBOT)};

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
        driver.start().onTrue(
            new InstantCommand(() -> s_Swerve.resetPvInitialization()).ignoringDisable(true));
        // intake forward
        driver.rightTrigger().whileTrue(intake.runIntakeMotor(1, .20));
        // intake backward
        driver.b().whileTrue(intake.runIndexerMotor(-.1));
        // intake and shoot as fast as possible
        driver.x().whileTrue(CommandFactory.passThroughShoot(shooter, intake));
        // toggle shooting while moving
        driver.a().toggleOnTrue(
            new ShootWhileMoving(s_Swerve, driver).alongWith(elevatorWrist.followPosition(() -> 0,
                () -> elevatorWrist.getAngleFromDistance(s_Swerve.getPose()))));

        // spit note currently in robot through shooter
        operator.x().whileTrue(CommandFactory.spit(shooter, intake));
        // shoot note to speaker after being intaked
        operator.rightTrigger().whileTrue(CommandFactory.shootSpeaker(shooter, intake));
        // set shooter to amp scoring preset position
        operator.start().onTrue(elevatorWrist.ampPosition());
        // set shooter to home preset position
        operator.back().onTrue(elevatorWrist.homePosition());
        // increment once through states list to next state
        operator.povRight().onTrue(Commands.runOnce(() -> {
            OperatorState.increment();
        }));
        // go back one through the states list to the previous state
        operator.povLeft().onTrue(Commands.runOnce(() -> {
            OperatorState.decrement();
        }));
        // go to current state as incremented through operator states list
        operator.a().onTrue(new MatchCommand<OperatorState.State>(List.of(OperatorState.State.kAmp),
            List.of(elevatorWrist.ampPosition()), OperatorState::getCurrentState));
        //
        operator.start().onTrue(Commands.runOnce(() -> {
            OperatorState.toggleManualMode();
        }));

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
