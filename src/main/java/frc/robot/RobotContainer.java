package frc.robot;

import java.util.List;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.MatchCommand;
import frc.lib.util.photon.PhotonCameraWrapper;
import frc.lib.util.photon.PhotonReal;
import frc.robot.Robot.RobotRunType;
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
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.DRIVER_ID);
    private final CommandXboxController operator = new CommandXboxController(Constants.OPERATOR_ID);
    // private final CommandXboxController test = new CommandXboxController(4);


    // Initialize AutoChooser Sendable
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    /* Subsystems */
    private Swerve s_Swerve;
    private Shooter shooter;
    private Intake intake;
    private PhotonCameraWrapper[] cameras;
    public ElevatorWrist elevatorWrist;
    public Climber climber;
    private LEDs leds = new LEDs(Constants.LEDConstants.LED_COUNT, Constants.LEDConstants.PWM_PORT);

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
        // autoChooser.addOption(resnickAuto, new ResnickAuto(s_Swerve));
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        // Configure the button bindings
        configureButtonBindings();
        Trigger gotNote = new Trigger(() -> !this.intake.getSensorStatus());
        gotNote.whileTrue(new FlashingLEDColor(leds, Color.kGreen));
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
        driver.leftTrigger().whileTrue(intake.runIntakeMotorNonStop(-1, -.20));

        /* Operator Buttons */
        // spit note currently in robot through shooter
        operator.x().whileTrue(CommandFactory.spit(shooter, intake));
        // reset apriltag vision
        operator.b().onTrue(new InstantCommand(() -> s_Swerve.resetPvInitialization()));
        // spin up shooter
        operator.leftTrigger().whileTrue(new ConditionalCommand(new StartEndCommand(() -> {
            climber.setLeftPower(SmartDashboard.getNumber("Left Climber Power", 0));
        }, () -> {
            climber.setLeftPower(0);
        }), shooter.shootSpeaker(),
            () -> OperatorState.getCurrentState() == OperatorState.State.kClimb));
        // shoot note to speaker after being intaked
        operator.rightTrigger().whileTrue(new ConditionalCommand(new StartEndCommand(() -> {
            climber.setLeftPower(SmartDashboard.getNumber("Left Climber Power", 0));
        }, () -> {
            climber.setLeftPower(0);
        }), CommandFactory.shootSpeaker(shooter, intake),
            () -> OperatorState.getCurrentState() == OperatorState.State.kClimb));
        // set shooter to home preset position
        operator.y().onTrue(elevatorWrist.homePosition());
        // increment once through states list to next state
        operator.povRight().onTrue(Commands.runOnce(() -> {
            OperatorState.increment();
        }));
        // go back one through the states list to the previous state
        operator.povLeft().onTrue(Commands.runOnce(() -> {
            OperatorState.decrement();
        }));
        // run action based on current state as incremented through operator states list
        operator.a()
            .whileTrue(new MatchCommand<OperatorState.State>(
                List.of(OperatorState.State.kAmp, OperatorState.State.kShootWhileMove,
                    OperatorState.State.kClimb),
                List.of(elevatorWrist.ampPosition(),
                    new ShootWhileMoving(s_Swerve, driver).alongWith(elevatorWrist.followPosition(
                        () -> Constants.ElevatorWristConstants.SetPoints.HOME_HEIGHT,
                        () -> elevatorWrist.getAngleFromDistance(s_Swerve.getPose()))),
                    elevatorWrist.ampPosition()),
                OperatorState::getCurrentState));
        // Toggle manual mode
        operator.start().onTrue(Commands.runOnce(() -> {
            OperatorState.toggleManualMode();
        }));
        // Flash LEDS to request amplify
        operator.povDown().onTrue(new FlashingLEDColor(leds, Color.kYellow));
        // Flash LEDs to request (TODO)
        operator.povUp().onTrue(new FlashingLEDColor(leds, Color.kPurple));
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
