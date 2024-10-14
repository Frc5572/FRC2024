package frc.robot;

import java.util.Map;
import edu.wpi.first.math.filter.Debouncer;
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
import frc.lib.util.FieldConstants;
import frc.lib.util.photon.PhotonCameraWrapper;
import frc.lib.util.photon.PhotonReal;
import frc.robot.Robot.RobotRunType;
import frc.robot.autos.JustShoot1;
import frc.robot.autos.P123;
import frc.robot.autos.P321;
import frc.robot.autos.P8765;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.FlashingLEDColor;
import frc.robot.commands.MovingColorLEDs;
import frc.robot.commands.ShootWhileMoving;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.LEDs;
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
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public ComplexWidget autoChooserWidget = mainDriverTab.add("Auto Chooser", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 6).withSize(3, 2);
    public GenericEntry operatorState = mainDriverTab.add("Operator State", "")
        .withWidget(BuiltInWidgets.kTextView).withPosition(8, 0).withSize(3, 2).getEntry();
    public GenericEntry operatorManualMode = RobotContainer.mainDriverTab.add("Manual Mode", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("true_color", 0xff00ffff, "false_color", 0xff770000))
        .withPosition(10, 6).withSize(2, 2).getEntry();
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
    public static SimpleWidget goToCenter =
        RobotContainer.mainDriverTab.add("Auto - Go To Centerline", false)
            .withWidget("Toggle Switch").withProperties(Map.of()).withPosition(7, 6).withSize(3, 1);
    public static SimpleWidget dumpNotes =
        RobotContainer.mainDriverTab.add("Auto - Dump Notes", false).withWidget("Toggle Switch")
            .withProperties(Map.of()).withPosition(7, 7).withSize(3, 1);
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);

    /* Subsystems */
    private Swerve s_Swerve;
    private Shooter shooter;
    private Intake intake;
    private PhotonCameraWrapper[] cameras;
    private ElevatorWrist elevatorWrist;
    private LEDs leds = new LEDs(Constants.LEDConstants.LED_COUNT, Constants.LEDConstants.PWM_PORT);
    // private PhotonCamera backLeftCamera = new PhotonCamera("back-left");


    private Trigger noteInIndexer = new Trigger(() -> this.intake.getIndexerBeamBrakeStatus())
        .debounce(0.25, Debouncer.DebounceType.kRising);
    private Trigger noteInIntake = new Trigger(() -> this.intake.getintakeBeamBrakeStatus())
        .debounce(0.25, Debouncer.DebounceType.kRising);

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
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
                // new PhotonReal(Constants.CameraConstants.FrontLeftFacingCamera.CAMERA_NAME,
                // Constants.CameraConstants.FrontLeftFacingCamera.CAMERA_IP),
                // Constants.CameraConstants.FrontLeftFacingCamera.KCAMERA_TO_ROBOT),
                new PhotonCameraWrapper(
                    new PhotonReal(Constants.CameraConstants.FrontRightFacingCamera.CAMERA_NAME,
                        Constants.CameraConstants.FrontRightFacingCamera.CAMERA_IP),
                    Constants.CameraConstants.FrontRightFacingCamera.KCAMERA_TO_ROBOT),
            // new PhotonCameraWrapper(
            // new PhotonReal(Constants.CameraConstants.BackLeftFacingCamera.CAMERA_NAME,
            // Constants.CameraConstants.BackLeftFacingCamera.CAMERA_IP),
            // Constants.CameraConstants.BackLeftFacingCamera.KCAMERA_TO_ROBOT)
            };
        // new PhotonCameraWrapper(
        // new PhotonReal(Constants.CameraConstants.BackRightFacingCamera.CAMERA_NAME),
        // Constants.CameraConstants.BackRightFacingCamera.KCAMERA_TO_ROBOT)};

        switch (runtimeType) {
            case kReal:
                shooter = new Shooter(new ShooterVortex());
                intake = new Intake(new IntakeIOFalcon());
                s_Swerve = new Swerve(new SwerveReal(), cameras);
                elevatorWrist = new ElevatorWrist(new ElevatorWristReal(), operator);
                break;
            case kSimulation:
                s_Swerve = new Swerve(new SwerveReal(), cameras);
                shooter = new Shooter(new ShooterIO() {});
                intake = new Intake(new IntakeIO() {});
                elevatorWrist = new ElevatorWrist(new ElevatorWristIO() {}, operator);
                break;
            default:
                s_Swerve = new Swerve(new SwerveIO() {}, cameras);
                shooter = new Shooter(new ShooterIO() {});
                intake = new Intake(new IntakeIO() {});
                elevatorWrist = new ElevatorWrist(new ElevatorWristIO() {}, operator);
        }

        autoChooser.setDefaultOption("Nothing", Commands.none());
        autoChooser.addOption("P123", new P123(s_Swerve, elevatorWrist, intake, shooter));
        autoChooser.addOption("P321", new P321(s_Swerve, elevatorWrist, intake, shooter));
        autoChooser.addOption("P8765", new P8765(s_Swerve, elevatorWrist, intake, shooter));
        autoChooser.addOption("Just Shoot 1",
            new JustShoot1(s_Swerve, elevatorWrist, intake, shooter));
        // autoChooser.addOption("P32", new P32(s_Swerve, elevatorWrist, intake, shooter));
        // autoChooser.addOption("P675", new P675(s_Swerve, elevatorWrist, intake, shooter));
        // autoChooser.addOption("P3675", new P3675(s_Swerve, elevatorWrist, intake, shooter));

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
        noteInIndexer.and(noteInIntake.negate()).onTrue(
            new FlashingLEDColor(leds, Constants.LEDConstants.INDEXER_COLOR).withTimeout(3));
        noteInIntake.and(noteInIndexer.negate())
            .onTrue(new FlashingLEDColor(leds, Constants.LEDConstants.INTAKE_COLOR).withTimeout(3));
        noteInIntake.and(noteInIndexer)
            .whileTrue(new FlashingLEDColor(leds, Constants.LEDConstants.ALERT_COLOR));


        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(() -> s_Swerve.resetFieldRelativeOffset()));
        driver.start().onTrue(
            new InstantCommand(() -> s_Swerve.resetPvInitialization()).ignoringDisable(true));
        // intake forward
        driver.rightTrigger().whileTrue(CommandFactory.newIntakeCommand(intake, elevatorWrist));
        // intake backward
        driver.leftTrigger().and(() -> elevatorWrist.getWristAngle().getDegrees() <= 24.0)
            .whileTrue(intake.runIntakeMotorNonStop(-1, -.20));

        /* Operator Buttons */
        // spit note currently in robot through shooter
        operator.x().whileTrue(CommandFactory.spit(shooter, intake));
        // reset apriltag vision
        operator.b().onTrue(new InstantCommand(() -> s_Swerve.resetPvInitialization()));
        // spin up shooter
        operator.leftTrigger().whileTrue(shooter.shootSpeaker());
        // operator.leftTrigger()
        // .whileTrue(new ShootWhileMoving(s_Swerve, driver, () -> s_Swerve.getPose(),
        // () -> FieldConstants
        // .allianceFlip(new Pose2d(FieldConstants.ampCenter, new Rotation2d()))
        // .getTranslation())
        // .alongWith(elevatorWrist.goToPosition(
        // Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
        // Constants.ElevatorWristConstants.SetPoints.HOME_ANGLE)));
        // shoot note to speaker after being intaked
        operator.rightTrigger().and(operator.leftTrigger().negate())
            .whileTrue(CommandFactory.shootSpeaker(shooter, intake));
        operator.rightTrigger().and(operator.leftTrigger()).whileTrue(intake.runIndexerMotor(1));
        // set shooter to home preset position
        operator.y().onTrue(elevatorWrist.homePosition());
        operator.y().and(elevatorWrist.elevatorAtAmp).and(noteInIndexer)
            .onTrue(intake.runIntakeMotorNonStop(0, -0.2).withTimeout(2.0)
                .until(new Trigger(() -> !this.intake.getIndexerBeamBrakeStatus()).debounce(.5)));

        // increment once through states list to next state
        operator.povRight().onTrue(Commands.runOnce(() -> {
            OperatorState.increment();
        }).ignoringDisable(true));
        // go back one through the states list to the previous state
        operator.povLeft().onTrue(Commands.runOnce(() -> {
            OperatorState.decrement();
        }).ignoringDisable(true));
        new Trigger(() -> OperatorState.getCurrentState() == OperatorState.State.kAmp)
            .and(new Trigger(() -> !this.intake.getIndexerBeamBrakeStatus()).debounce(1.0))
            .onTrue(elevatorWrist.homePosition());
        // run action based on current state as incremented through operator states list
        operator.a().whileTrue(new SelectCommand<OperatorState.State>(Map.of(
            //
            OperatorState.State.kSpeaker,
            elevatorWrist.speakerPreset()
                .alongWith(new TeleopSwerve(s_Swerve, driver, Constants.Swerve.isFieldRelative,
                    Constants.Swerve.isOpenLoop)),
            //
            OperatorState.State.kAmp,
            Commands.either(elevatorWrist.ampPosition(), Commands.none(), noteInIndexer)
                .alongWith(new TeleopSwerve(s_Swerve, driver, Constants.Swerve.isFieldRelative,
                    Constants.Swerve.isOpenLoop)),
            //
            OperatorState.State.kShootWhileMove,
            new ShootWhileMoving(s_Swerve, driver, () -> s_Swerve.getPose(),
                () -> FieldConstants.allianceFlip(FieldConstants.Speaker.centerSpeakerOpening)
                    .getTranslation())
                        .alongWith(elevatorWrist.followPosition(
                            () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                            () -> elevatorWrist.getAngleFromDistance(s_Swerve.getPose())
                                .plus(Rotation2d.fromDegrees(0.0)))),
            //
            OperatorState.State.kPost,
            new TurnToAngle(s_Swerve, Rotation2d.fromDegrees(25)).alongWith(elevatorWrist
                .followPosition(() -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                    () -> Constants.ElevatorWristConstants.SetPoints.PODIUM_ANGLE)),
            //
            OperatorState.State.kClimb,
            Commands
                .sequence(elevatorWrist.ampPosition(),
                    Commands.runOnce(() -> OperatorState.enableManualMode()))
                .alongWith(new TeleopSwerve(s_Swerve, driver, Constants.Swerve.isFieldRelative,
                    Constants.Swerve.isOpenLoop))),
            OperatorState::getCurrentState));

        // Toggle manual mode
        operator.start().onTrue(Commands.runOnce(() -> {
            OperatorState.toggleManualMode();
        }).ignoringDisable(true));
        // Flash LEDS to request amplify
        operator.povUp().onTrue(new FlashingLEDColor(leds, Color.kGold).withTimeout(5));
        // Flash LEDs to request
        operator.povDown().onTrue(new FlashingLEDColor(leds, Color.kBlue).withTimeout(5));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        OperatorState.setState(OperatorState.State.kShootWhileMove);
        Command autocommand = autoChooser.getSelected();
        return autocommand;
    }

}
