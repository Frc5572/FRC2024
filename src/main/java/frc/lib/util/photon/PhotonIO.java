package frc.lib.util.photon;

import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

public abstract class PhotonIO {

    public static class PhotonInputs implements LoggableInputs {

        public byte[] rawBytes;
        public PhotonPipelineResult result;
        public String versionString;
        public double timeSinceLastHeartbeat;
        public double[] cameraMatrix;
        public double[] distCoeffs;
        public String name;

        public PhotonInputs() {
            rawBytes = new byte[0];
            result = null;
            versionString = "";
            timeSinceLastHeartbeat = -1;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("rawBytes", rawBytes);
            table.put("versionString", versionString);
            table.put("timeSinceLastHeartbeat", timeSinceLastHeartbeat);
            table.put("cameraMatrix", cameraMatrix);
            table.put("distCoeffs", distCoeffs);
        }

        @Override
        public void fromLog(LogTable table) {
            this.rawBytes = table.get("rawBytes", rawBytes);
            Packet p = new Packet(rawBytes);
            this.result = PhotonPipelineResult.serde.unpack(p);

            this.versionString = table.get("versionString", versionString);
            this.timeSinceLastHeartbeat =
                table.get("timeSinceLastHeartbeat", timeSinceLastHeartbeat);
            this.cameraMatrix = table.get("cameraMatrix", cameraMatrix);
            this.distCoeffs = table.get("distCoeffs", distCoeffs);
        }

        public Optional<Matrix<N3, N3>> getCameraMatrix() {
            if (cameraMatrix != null && cameraMatrix.length == 9) {
                return Optional.of(MatBuilder.fill(Nat.N3(), Nat.N3(), cameraMatrix));
            } else
                return Optional.empty();
        }

        public Optional<Matrix<N5, N1>> getDistCoeffs() {
            if (distCoeffs != null && distCoeffs.length == 5) {
                return Optional.of(MatBuilder.fill(Nat.N5(), Nat.N1(), distCoeffs));
            } else
                return Optional.empty();
        }

    }

    private final String name;

    public PhotonIO(String name) {
        this.name = name;
    }

    public void updateInputs(PhotonInputs inputs) {
        inputs.name = name;
    }

}
