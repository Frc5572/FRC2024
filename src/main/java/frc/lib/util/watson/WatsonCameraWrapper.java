package frc.lib.util.watson;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.util.watson.WatsonIO.WatsonInputs;
import frc.robot.Constants;
import frc.robot.Robot;

public class WatsonCameraWrapper {

    public WatsonIO io;
    public WatsonInputs inputs = new WatsonInputs();
    private Translation2d offset;

    public WatsonCameraWrapper(WatsonIO io, Translation2d robotToCam) {
        this.io = io;
        this.offset = robotToCam;
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Watson/" + inputs.name, inputs);
    }

    public static class VisionObservation {
        public int fudicialId;
        public Pose2d robotPose;
        public Matrix<N3, N1> stdDev;

        /** All fields constructor. */
        public VisionObservation(int fudicialId, Pose2d robotPose, Matrix<N3, N1> stdDev) {
            this.fudicialId = fudicialId;
            this.robotPose = robotPose;
            this.stdDev = stdDev;
        }
    }

    public Optional<VisionObservation> getMultiTagPose(Rotation2d gyroYaw, double speed_mps) {
        if (this.inputs.result == null || !this.inputs.isMultiTag) {
            return Optional.empty();
        }

        double xy_dev_base = Constants.CameraConstants.XY_STD_DEV_COEFF;
        if (Robot.inAuto) {
            xy_dev_base = Constants.CameraConstants.XY_STD_DEV_AUTO_COEFF;
        }

        Pose2d res = new Pose2d(this.inputs.result.minus(this.offset.rotateBy(gyroYaw)), gyroYaw);
        this.inputs.rawBytes = new byte[] {};
        return Optional.of(new VisionObservation(0, res,
            VecBuilder.fill(
                xy_dev_base + Constants.CameraConstants.SPEED_STD_DEV_COEFF * speed_mps
                    + Constants.CameraConstants.REPROJ_STD_DEV_COEFF
                        * this.inputs.reprojectionError,
                xy_dev_base + Constants.CameraConstants.SPEED_STD_DEV_COEFF * speed_mps
                    + Constants.CameraConstants.REPROJ_STD_DEV_COEFF
                        * this.inputs.reprojectionError,
                Constants.CameraConstants.THETA_STD_DEV_COEFF)));
    }

}
