package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoral;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.Deadzone;
import frc.robot.Robot.RobotRunType;
import frc.robot.subsystems.swerve.drive.Swerve;
import frc.robot.subsystems.swerve.drive.SwerveNavX;
import frc.robot.subsystems.swerve.drive.SwerveSim;
import frc.robot.subsystems.swerve.mod.SwerveModuleSim;
import frc.robot.subsystems.swerve.mod.SwerveModuleTalonAngle;
import frc.robot.subsystems.swerve.mod.SwerveModuleTalonDrive;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(Constants.driverId);

    private SwerveDriveSimulation driveSimulation;

    /* Subsystems */
    private Swerve s_Swerve;

    /**
     */
    public RobotContainer(RobotRunType runtimeType) {
        if (runtimeType == RobotRunType.kSimulation) {
            SimulatedArena.overrideSimulationTimings(Units.Seconds.of(0.02), 5);
        }

        switch (runtimeType) {
            case kReal:
                s_Swerve = new Swerve(new SwerveNavX(), SwerveModuleTalonAngle::new,
                    SwerveModuleTalonDrive::new);
                break;
            case kSimulation:
                driveSimulation =
                    new SwerveDriveSimulation(Constants.Swerve.getMapleConfig(),
                        new Pose2d(3, 3, Rotation2d.kZero));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                s_Swerve = new Swerve(new SwerveSim(driveSimulation),
                    (i, config) -> {
                        var sim = new SwerveModuleSim(config, driveSimulation.getModules()[i]);
                        return Pair.of(sim, sim);
                    });
                break;
            default:
                // s_Swerve = new Swerve(new SwerveIO() {}, cameras, viz);
        }

        double maxSpeed = Constants.Swerve.maxLinearSpeed.in(Units.MetersPerSecond);

        s_Swerve.setDefaultCommand(s_Swerve.drive(() -> {
            double forwardNormalized = -driver.getLeftY();
            double leftNormalized = -driver.getLeftX();
            double turnNormalized = -driver.getRightX();

            double forwardDeadband = Deadzone.applyDeadzone(forwardNormalized);
            double leftDeadband = Deadzone.applyDeadzone(leftNormalized);
            double turnDeadband = Deadzone.applyDeadzone(turnNormalized);

            double forward =
                forwardDeadband * forwardDeadband * Math.signum(forwardDeadband) * maxSpeed;
            double left = leftDeadband * leftDeadband * Math.signum(leftDeadband) * maxSpeed;
            double turn = turnDeadband * turnDeadband * Math.signum(turnDeadband) * maxSpeed;

            return new ChassisSpeeds(forward, left, turn);
        }, true, true));

        configureButtonBindings(runtimeType);
    }

    /**
     * Use this method to vol your button->command mappings. Buttons can be created by instantiating
     * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings(RobotRunType runtimeType) {
        if (runtimeType == RobotRunType.kSimulation) {
            driver.a().onTrue(new InstantCommand(() -> {
                SimulatedArena.getInstance()
                    .addGamePiece(new ReefscapeCoral(new Pose2d(2, 2, Rotation2d.kZero)));
            }));
        }
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

    /** Start simulation */
    public void startSimulation() {
        if (driveSimulation != null) {
            SimulatedArena.getInstance().resetFieldForAuto();
        }
    }

    /**
     * Update simulation
     */
    public void updateSimulation() {
        if (driveSimulation != null) {
            SimulatedArena.getInstance().simulationPeriodic();
            Logger.recordOutput("simulatedPose", driveSimulation.getSimulatedDriveTrainPose());
            Logger.recordOutput("FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
            Logger.recordOutput("FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            Logger.recordOutput("FieldSimulation/StackedAlgae",
                ReefscapeCoralAlgaeStack.getStackedAlgaePoses());
            Logger.recordOutput("FieldSimulation/StackedCoral",
                ReefscapeCoralAlgaeStack.getStackedCoralPoses());
        }
    }

}
