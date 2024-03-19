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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.watson.WatsonIO.WatsonInputs;
import frc.robot.Constants;

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

        if (inputs.result != null) {
            SmartDashboard.putNumber("watson/x", inputs.result.getX());
            SmartDashboard.putNumber("watson/y", inputs.result.getY());
        }
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

    public Optional<VisionObservation> getMultiTagPose(Rotation2d gyroYaw) {
        if (this.inputs.result == null || !this.inputs.isMultiTag) {
            return Optional.empty();
        }
        Pose2d res = new Pose2d(this.inputs.result.minus(this.offset.rotateBy(gyroYaw)), gyroYaw);
        this.inputs.rawBytes = new byte[] {};
        return Optional.of(new VisionObservation(0, res,
            VecBuilder.fill(Constants.CameraConstants.XY_STD_DEV_COEFF,
                Constants.CameraConstants.XY_STD_DEV_COEFF,
                Constants.CameraConstants.THETA_STD_DEV_COEFF)));
    }

}
