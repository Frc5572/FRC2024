package frc.robot.subsystems.vision;

import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

public interface CameraIO {

    public static class CameraInputs implements LoggableInputs {

        public PhotonPipelineResult result;
        public String versionString;
        public double timeSinceLastHeartbeat;
        public Optional<Matrix<N3, N3>> cameraMatrix = Optional.empty();
        public Optional<Matrix<N5, N1>> distCoeffs = Optional.empty();
        public String name;

        /**
         * Default Constructor.
         */
        public CameraInputs() {
            result = null;
            versionString = "";
            timeSinceLastHeartbeat = -1;
        }

        @Override
        public void toLog(LogTable table) {
            if (result == null) {
                table.put("rawBytes", EMPTY_BYTES);
            } else {
                // https://github.com/PhotonVision/photonvision/blob/471c90e8fabc55aad039bb712df7dcc11d12fc6b/photon-core/src/main/java/org/photonvision/common/dataflow/networktables/NTDataPublisher.java#L144
                var rawBytes = new Packet(1024);
                PhotonPipelineResult.serde.pack(rawBytes, result);
                table.put("rawBytes", rawBytes.getData());
            }
            if (cameraMatrix.isPresent()) {
                table.put("cameraMatrix", cameraMatrix.get().getData());
            } else {
                table.put("cameraMatrix", new double[0]);
            }
            if (distCoeffs.isPresent()) {
                table.put("distCoeffs", distCoeffs.get().getData());
            } else {
                table.put("distCoeffs", new double[0]);
            }
        }

        private static final byte[] EMPTY_BYTES = new byte[0];

        @Override
        public void fromLog(LogTable table) {
            byte[] rawBytes = table.get("rawBytes", EMPTY_BYTES);
            if (rawBytes.length > 0) {
                Packet p = new Packet(rawBytes);
                this.result = PhotonPipelineResult.serde.unpack(p);
            } else {
                this.result = null;
            }

            double[] data = table.get("cameraMatrix", new double[0]);
            if (data.length == 0) {
                this.cameraMatrix = Optional.empty();
            } else {
                this.cameraMatrix = Optional.of(new Matrix<>(N3.instance, N3.instance, data));
            }
            data = table.get("distCoeffs", new double[0]);
            if (data.length == 0) {
                this.distCoeffs = Optional.empty();
            } else {
                this.distCoeffs = Optional.of(new Matrix<>(N5.instance, N1.instance, data));
            }
        }

    }

    public static class Empty implements CameraIO {

        public Empty(CameraConstants constants) {}

        @Override
        public void updateInputs(CameraInputs inputs) {}

    }

    public void updateInputs(CameraInputs inputs);

}
