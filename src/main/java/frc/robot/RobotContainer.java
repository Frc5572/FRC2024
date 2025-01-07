package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.swerve.drive.Swerve;
import frc.robot.subsystems.swerve.drive.SwerveSim;
import frc.robot.subsystems.swerve.mod.SwerveModuleSim;


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

    /* Subsystems */
    private Swerve s_Swerve;
    private LEDs leds = new LEDs(Constants.LEDConstants.LED_COUNT, Constants.LEDConstants.PWM_PORT);

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        if (runtimeType == RobotRunType.kSimulation) {
            SimulatedArena.overrideSimulationTimings(Units.Seconds.of(0.02), 5);
        }

        switch (runtimeType) {
            case kReal:
                // s_Swerve = new Swerve(new SwerveReal(), cameras, viz);
                break;
            case kSimulation:
                var driveSimulation = new SwerveDriveSimulation(Constants.Swerve.mapleSimConfig,
                    new Pose2d(3, 3, Rotation2d.kZero));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                s_Swerve = new Swerve(new SwerveSim(driveSimulation.getGyroSimulation()),
                    Constants.Swerve.config.modules(), (i, config) -> {
                        var sim = new SwerveModuleSim(config, driveSimulation.getModules()[i]);
                        return Pair.of(sim, sim);
                    });
                break;
            default:
                // s_Swerve = new Swerve(new SwerveIO() {}, cameras, viz);
        }

        configureButtonBindings();
    }

    /**
     * Use this method to vol your button->command mappings. Buttons can be created by instantiating
     * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        return null;
    }

    /**
     * Update viz
     */
    public void updateViz() {

    }

    /**
     * Update simulation
     */
    public void updateSimulation() {
        SimulatedArena.getInstance().simulationPeriodic();
    }

}
