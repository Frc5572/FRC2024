package frc.robot;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.photon.PhotonCameraWrapper;
import frc.lib.util.photon.PhotonReal;
import frc.robot.Robot.RobotRunType;
import frc.robot.autos.ChoreoTests;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.FlashingLEDColor;
import frc.robot.commands.MovingColorLEDs;
import frc.robot.commands.ShootWhileMoving;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.LEDs;
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
        .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 6).withSize(3, 2);
    public GenericEntry operatorState =
        mainDriverTab.add("Operator State", OperatorState.getCurrentState().displayName)
            .withWidget(BuiltInWidgets.kTextView).withPosition(8, 0).withSize(3, 2).getEntry();
    public GenericEntry operatorManualMode = RobotContainer.mainDriverTab.add("Manual Mode", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("true_color", 0xff00ffff, "false_color", 0xff770000))
        .withPosition(7, 6).withSize(2, 2).getEntry();
    public static GenericEntry readyShoot = RobotContainer.mainDriverTab
        .add("Ready To Shoot", false).withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("true_color", 0xff00ffff, "false_color", 0xff770000))
        .withPosition(10, 2).withSize(3, 2).getEntry();
    public SimpleWidget backLeftCameraWidget = RobotContainer.mainDriverTab
        .add("Back Left Camera", "/CameraPublisher/back-left_Port_1182_Output_MJPEG_Server")
        .withWidget(BuiltInWidgets.kCameraStream)
        .withProperties(Map.of("topic", "/CameraPublisher/back-left_Port_1182_Output_MJPEG_Server"))
        .withPosition(12, 4).withSize(7, 6);

    public static final SendableChooser<Integer> numNoteChooser = new SendableChooser<>();
    // public ComplexWidget numNoteChooserrWidget =
    // mainDriverTab.add("Number of Additional Auto Notes", numNoteChooser)
    // .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(7, 6).withSize(3, 2);
    public SimpleWidget fmsInfo =
        RobotContainer.mainDriverTab.add("FMS Info", 0).withWidget("FMSInfo")
            .withProperties(Map.of("topic", "/FMSInfo")).withPosition(3, 4).withSize(6, 2);
    public GenericEntry matchTime = RobotContainer.mainDriverTab.add("Match Time", 0)
        .withWidget("Match Time").withProperties(Map.of("time_display_mode", "Minutes and Seconds"))
        .withPosition(0, 4).withSize(3, 2).getEntry();
    public SimpleWidget voltageInfo =
        RobotContainer.mainDriverTab.add("Battery Voltage", 0).withWidget("Voltage View")
            .withProperties(Map.of("topic", "/AdvantageKit/SystemStats/BatteryVoltage"))
            .withPosition(0, 6).withSize(4, 2);
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);
    // private final CommandXboxController test = new CommandXboxController(4);



    /* Subsystems */
    private Swerve s_Swerve;
    private Shooter shooter;
    private Intake intake;
    private PhotonCameraWrapper[] cameras;
    private ElevatorWrist elevatorWrist;
    public Climber climber;
    private LEDs leds = new LEDs(Constants.LEDConstants.LED_COUNT, Constants.LEDConstants.PWM_PORT);
    // private PhotonCamera backLeftCamera = new PhotonCamera("back-left");


    private Trigger gotNote = new Trigger(() -> !this.intake.getSensorStatus());
    private Trigger climbState = new Trigger(() -> false);
    // new Trigger(() -> OperatorState.getCurrentState() == OperatorState.State.kClimb);
    private Trigger mannualMode = new Trigger(() -> OperatorState.manualModeEnabled());
    private Trigger atHome = new Trigger(() -> elevatorWrist.elevatorAtHome());

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        autoChooser.setDefaultOption("Wait 1 Second", "wait");
        autoChooser.addOption("P123", "P123");
        autoChooser.addOption("P321", "P321");
        autoChooser.addOption("P32", "P32");
        autoChooser.addOption("Resnick 5", "Resnick 5");
        // autoChooser.addOption("Resnick 3", "Resnick 3");
        // autoChooser.addOption("Resnick 4", "Resnick 4");
        numNoteChooser.setDefaultOption("0", 0);
        for (int i = 0; i < 7; i++) {
            numNoteChooser.addOption(String.valueOf(i), i);
        }
        // backLeftCamera.setDriverMode(true);
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
        leds.setDefaultCommand(new MovingColorLEDs(leds, Color.kRed, 4, false));
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to vol your button->command mappings. Buttons can be created by instantiating
     * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        gotNote.onTrue(new FlashingLEDColor(leds, Color.kGreen).withTimeout(3));
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));
        driver.start().onTrue(
            new InstantCommand(() -> s_Swerve.resetPvInitialization()).ignoringDisable(true));
        // intake forward
        driver.rightTrigger().whileTrue(intake.runIntakeMotor(1, .20));
        // intake backward
        driver.leftTrigger().whileTrue(intake.runIntakeMotorNonStop(-1, -.20));

        /* Operator Buttons */
        // spit note currently in robot through shooter
        operator.x().whileTrue(CommandFactory.spit(shooter, intake));
        // reset apriltag vision
        operator.b().onTrue(new InstantCommand(() -> s_Swerve.resetPvInitialization()));
        // spin up shooter
        operator.leftTrigger().and(climbState).and(mannualMode).whileTrue(Commands.startEnd(() -> {
            climber.setLeftPower(Constants.ClimberConstants.CLIMBER_POWER);
        }, () -> {
            climber.setLeftPower(0);
        }));
        operator.leftTrigger().and(climbState.negate()).whileTrue(shooter.shootSpeaker());
        // shoot note to speaker after being intaked
        operator.rightTrigger().and(climbState).and(mannualMode).whileTrue(Commands.startEnd(() -> {
            climber.setRightPower(Constants.ClimberConstants.CLIMBER_POWER);
        }, () -> {
            climber.setRightPower(0);
        }));
        operator.rightTrigger().and(climbState.negate())
            .whileTrue(CommandFactory.shootSpeaker(shooter, intake));
        operator.rightTrigger().and(climbState).and(mannualMode.negate())
            .whileTrue(Commands.startEnd(() -> {
                climber.setRightPower(Constants.ClimberConstants.CLIMBER_POWER);
                climber.setLeftPower(Constants.ClimberConstants.CLIMBER_POWER);
            }, () -> {
                climber.setRightPower(0);
                climber.setLeftPower(0);
            }));
        // set shooter to home preset position
        operator.y().onTrue(elevatorWrist.homePosition());
        // increment once through states list to next state
        operator.povRight().onTrue(Commands.runOnce(() -> {
            OperatorState.increment();
        }).ignoringDisable(true));
        // go back one through the states list to the previous state
        operator.povLeft().onTrue(Commands.runOnce(() -> {
            OperatorState.decrement();
        }).ignoringDisable(true));
        // run action based on current state as incremented through operator states list
        operator.a().whileTrue(new SelectCommand<OperatorState.State>(Map.of(
            //
            // OperatorState.State.kAmp,
            // Commands
            // .either(elevatorWrist.ampPosition(), Commands.none(),
            // gotNote.debounce(0.5, Debouncer.DebounceType.kFalling))
            // .alongWith(new TeleopSwerve(s_Swerve, driver, true, false)),
            //
            // OperatorState.State.kClimb,
            // elevatorWrist.climbPosition()
            // .alongWith(new TeleopSwerve(s_Swerve, driver, true, false)),
            //
            OperatorState.State.kShootWhileMove,
            new ShootWhileMoving(s_Swerve, driver).alongWith(elevatorWrist.followPosition(
                () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT, () -> elevatorWrist
                    .getAngleFromDistance(s_Swerve.getPose()).plus(Rotation2d.fromDegrees(0.0))))
        //
        ), OperatorState::getCurrentState));

        /*
         * <OperatorState.State>( List.of(OperatorState.State.kAmp,
         * OperatorState.State.kShootWhileMove, OperatorState.State.kClimb),
         * List.of(elevatorWrist.ampPosition(), new ShootWhileMoving(s_Swerve,
         * driver).alongWith(elevatorWrist.followPosition( () ->
         * Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT, () ->
         * elevatorWrist.getAngleFromDistance(s_Swerve.getPose()))), elevatorWrist.ampPosition()),
         * )));
         */
        // Toggle manual mode
        operator.start().onTrue(Commands.runOnce(() -> {
            OperatorState.toggleManualMode();
        }).ignoringDisable(true));
        // Flash LEDS to request amplify
        operator.povUp().onTrue(new FlashingLEDColor(leds, Color.kGold).withTimeout(5));
        // Flash LEDs to request (TODO)
        operator.povDown().onTrue(new FlashingLEDColor(leds, Color.kBlue).withTimeout(5));



        // test.leftTrigger().whileTrue(Commands.startEnd(() -> {
        // climber.setLeftPower(-1);
        // }, () -> {
        // climber.setLeftPower(0);
        // }));
        // // shoot note to speaker after being intaked
        // test.rightTrigger().whileTrue(Commands.startEnd(() -> {
        // climber.setRightPower(-1);
        // }, () -> {
        // climber.setRightPower(0);
        // }));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        return ChoreoTests.testPath(s_Swerve, intake, elevatorWrist)
            .alongWith(shooter.shootSpeaker());
    }

}
