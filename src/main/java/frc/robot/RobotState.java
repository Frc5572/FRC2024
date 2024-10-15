package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class RobotState {

    public record OdometryObservation(SwerveDriveWheelPositions wheelPositions,
        Rotation2d gyroAngle, double timestamp) {
    }

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    }

    private static final double poseBufferSizeSeconds = 2.0;

    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
        TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
    private final Matrix<N3, N1> qStdDevs =
        new Matrix<>(VecBuilder.fill(Math.pow(0.003, 2), Math.pow(0.003, 2), Math.pow(0.0002, 2)));
    // Odometry
    private final SwerveDriveKinematics kinematics;
    private SwerveDriveWheelPositions lastWheelPositions =
        new SwerveDriveWheelPositions(new SwerveModulePosition[] {new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()});
    private Rotation2d lastGyroAngle = new Rotation2d();
    private Twist2d robotVelocity = new Twist2d();
    private Twist2d trajectoryVelocity = new Twist2d();

    private RobotState() {
        this.kinematics = Constants.Swerve.swerveKinematics;
    }

    public void addOdometryObservation(OdometryObservation observation) {
        // TODO
        // https://github.com/Mechanical-Advantage/RobotCode2024/blob/a025615a52193b7709db7cf14c51c57be17826f2/src/main/java/org/littletonrobotics/frc2024/RobotState.java#L160
    }

    public void addVisionObservation(VisionObservation observation) {
        // TODO
        // https://github.com/Mechanical-Advantage/RobotCode2024/blob/a025615a52193b7709db7cf14c51c57be17826f2/src/main/java/org/littletonrobotics/frc2024/RobotState.java#L181
    }

}
