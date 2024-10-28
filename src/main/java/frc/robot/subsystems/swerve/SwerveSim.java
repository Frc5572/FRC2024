package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import frc.lib.sim.SimulatedPumbaa;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleSim;
import frc.robot.Constants;

/**
 * Real Class for Swerve
 */
public class SwerveSim implements SwerveIO {

    private SwerveModule[] swerveMods;
    private final AnalogGyro m_gyro = new AnalogGyro(0);
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()};

    private Pose2d currentPose = new Pose2d();
    private final SimulatedPumbaa pumbaa;

    /**
     * Real Swerve Initializer
     */
    public SwerveSim(SimulatedPumbaa pumbaa) {
        this.pumbaa = pumbaa;
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {
        m_gyroSim.setAngle(getHeading().getDegrees());
        if (Constants.Swerve.invertGyro) {
            inputs.yaw = (float) -m_gyroSim.getAngle();
        } else {
            inputs.yaw = (float) m_gyroSim.getAngle();
        }
        // inputs.pitch = 0;
        // inputs.roll = 0;

    }

    /**
     * Create Swerve Module
     *
     * @param moduleNumber Module Number
     * @return Swerve Module
     */
    public SwerveModule createSwerveModule(int moduleNumber) {
        SwerveModuleSim modIO = new SwerveModuleSim();
        modIO.setModNumber(moduleNumber);
        SwerveModule mod =
            new SwerveModule(moduleNumber, Rotation2d.fromRotations(Math.random()), modIO);
        return mod;
    }

    @Override
    public SwerveModule[] createModules() {
        swerveMods = new SwerveModule[] {createSwerveModule(0), createSwerveModule(1),
            createSwerveModule(2), createSwerveModule(3)};
        return swerveMods;
    }

    /**
     * Get Heading
     *
     * @return Get robot heading
     */
    private Rotation2d getHeading() {
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            modulePositions[moduleIndex] = swerveMods[moduleIndex].getPosition();
            moduleDeltas[moduleIndex] = new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }
        Twist2d twist = Constants.Swerve.swerveKinematics.toTwist2d(moduleDeltas);
        this.currentPose = this.currentPose.exp(twist);
        this.pumbaa.setPose(this.currentPose);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        return rawGyroRotation;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.currentPose = pose;
        this.pumbaa.setPose(this.currentPose);
    }

}
