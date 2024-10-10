package frc.lib.util.photon;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import org.apache.http.HttpEntity;
import org.apache.http.client.methods.CloseableHttpResponse;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.mime.MultipartEntityBuilder;
import org.apache.http.entity.mime.content.FileBody;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** PhotonCamera modified to work with AdvantageKit */
public class LoggedPhotonCamera extends PhotonCamera {

    /** Inputs for LoggedPhotonCamera */
    public static class PhotonCameraInputs implements LoggableInputs, Cloneable {
        public PhotonPipelineResult result = null;
        public Optional<Matrix<N3, N3>> cameraMatrix = Optional.empty();
        public Optional<Matrix<N5, N1>> distCoeffs = Optional.empty();
        public String name = "";

        @Override
        public void toLog(LogTable table) {
            if (result == null) {
                var rawBytes = new byte[0];
                table.put("rawBytes", rawBytes);
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

        @Override
        public void fromLog(LogTable table) {
            var rawBytes = table.get("rawBytes", new byte[0]);
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

    private final PhotonCameraInputs inputs = new PhotonCameraInputs();

    /** Create PhotonCamera with a given name and IP. */
    public LoggedPhotonCamera(String cameraName, String cameraIP) {
        super(cameraName);
        inputs.name = cameraName;
        new Thread(() -> {
            Timer timer = new Timer();
            while (true) {
                if (timer.advanceIfElapsed(10.0) && !isPhotonOk.get()) {
                    try {
                        uploadSettings(cameraIP + ":5800",
                            new File(Filesystem.getDeployDirectory().getAbsoluteFile(),
                                "photon-configs/" + cameraName + ".zip"));
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
                Thread.yield();
            }
        }).start();
    }

    /** Update inputs for this camera. */
    public void periodic() {
        this.inputs.result = super.getLatestResult();
        this.inputs.cameraMatrix = super.getCameraMatrix();
        this.inputs.distCoeffs = super.getDistCoeffs();
        Logger.processInputs("Camera_" + this.inputs.name, inputs);
        this.isPhotonOk.set(this.inputs.distCoeffs.isPresent());
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        return this.inputs.result;
    }

    @Override
    public Optional<Matrix<N3, N3>> getCameraMatrix() {
        return this.inputs.cameraMatrix;
    }

    @Override
    public Optional<Matrix<N5, N1>> getDistCoeffs() {
        return this.inputs.distCoeffs;
    }

    private boolean uploadSettings(String ip, File file) throws IOException {
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            HttpPost postReq = new HttpPost("http://" + ip + "/api/settings");
            HttpEntity entity =
                MultipartEntityBuilder.create().addPart("data", new FileBody(file)).build();
            postReq.setEntity(entity);
            try (CloseableHttpResponse response = httpClient.execute(postReq)) {
                SmartDashboard.putString("uploadSettings/" + this.inputs.name + "/status",
                    response.getStatusLine().getStatusCode() + ": "
                        + response.getStatusLine().getReasonPhrase());
                var ent = response.getEntity();
                if (ent != null) {
                    try (InputStream stream = ent.getContent()) {
                        String text = new String(stream.readAllBytes(), StandardCharsets.UTF_8);
                        SmartDashboard.putString("uploadSettings/" + this.inputs.name + "/content",
                            text);
                    }
                } else {
                    SmartDashboard.putString("uploadSettings/" + this.inputs.name + "/content",
                        "null");
                }
                return response.getStatusLine().getStatusCode() == 200;
            }
        }
    }

    private AtomicBoolean isPhotonOk = new AtomicBoolean(false);

}
