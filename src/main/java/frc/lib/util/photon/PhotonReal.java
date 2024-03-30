package frc.lib.util.photon;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import org.apache.http.HttpEntity;
import org.apache.http.client.methods.CloseableHttpResponse;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.mime.MultipartEntityBuilder;
import org.apache.http.entity.mime.content.FileBody;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.common.dataflow.structures.PacketSerde;
import org.photonvision.common.networktables.PacketSubscriber;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents an actual camera that is connected to PhotonVision. Based on {@link PhotonCamera}. */
public class PhotonReal extends PhotonIO implements AutoCloseable {
    public static final String kTableName = "photonvision";
    private static int InstanceCount = 0;

    private final NetworkTable cameraTable;
    private PacketSubscriber<PhotonPipelineResultIntermediate> resultSubscriber;
    private IntegerSubscriber heartbeatEntry;
    private StringSubscriber versionEntry;
    private DoubleArraySubscriber cameraIntrinsicsSubscriber;
    private DoubleArraySubscriber cameraDistortionSubscriber;

    private long prevHeartbeatValue = -1;
    private double prevHeartbeatChangeTime = 0;

    private static class PhotonPipelineResultIntermediate {
        public PhotonPipelineResult result;
        public byte[] rawBytes;

        public PhotonPipelineResultIntermediate() {
            result = new PhotonPipelineResult();
            rawBytes = new byte[0];
        }

        public PhotonPipelineResultIntermediate(PhotonPipelineResult result, byte[] rawBytes) {
            this.result = result;
            this.rawBytes = rawBytes;
        }
    }

    private static class PhotonResultIntermediateBuilder
        implements PacketSerde<PhotonPipelineResultIntermediate> {

        @Override
        public int getMaxByteSize() {
            // This uses dynamic packets so it doesn't matter
            return -1;
        }

        @Override
        public void pack(Packet packet, PhotonPipelineResultIntermediate value) {
            PhotonPipelineResult.serde.pack(packet, value.result);
        }

        @Override
        public PhotonPipelineResultIntermediate unpack(Packet packet) {
            byte[] data = packet.getData();
            var result = PhotonPipelineResult.serde.unpack(packet);
            return new PhotonPipelineResultIntermediate(result, data);
        }

    }

    private boolean uploadSettings(String ip, File file) throws IOException {
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            HttpPost postReq = new HttpPost("http://" + ip + "/api/settings");
            HttpEntity entity =
                MultipartEntityBuilder.create().addPart("data", new FileBody(file)).build();
            postReq.setEntity(entity);
            try (CloseableHttpResponse response = httpClient.execute(postReq)) {
                SmartDashboard.putString("uploadSettings/" + this.name + "/status",
                    response.getStatusLine().getStatusCode() + ": "
                        + response.getStatusLine().getReasonPhrase());
                var ent = response.getEntity();
                if (ent != null) {
                    try (InputStream stream = ent.getContent()) {
                        String text = new String(stream.readAllBytes(), StandardCharsets.UTF_8);
                        SmartDashboard.putString("uploadSettings/" + this.name + "/content", text);
                    }
                } else {
                    SmartDashboard.putString("uploadSettings/" + this.name + "/content", "null");
                }
                return response.getStatusLine().getStatusCode() == 200;
            }
        }
    }

    /**
     * Constructs a PhotonReal from a root table.
     *
     * @param instance The NetworkTableInstance to pull data from.
     * @param cameraName The name of the camera, as seen in the UI.
     */
    public PhotonReal(NetworkTableInstance instance, String cameraName, String cameraIP) {
        super(cameraName);
        try {
            uploadSettings(cameraIP + ":5800",
                new File(Filesystem.getDeployDirectory().getAbsoluteFile(),
                    "photon-configs/" + cameraName + ".zip"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        var photonvision_root_table = instance.getTable(kTableName);
        this.cameraTable = photonvision_root_table.getSubTable(cameraName);
        var rawBytesEntry = cameraTable.getRawTopic("rawBytes").subscribe("rawBytes", new byte[] {},
            PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
        resultSubscriber = new PacketSubscriber<PhotonPipelineResultIntermediate>(rawBytesEntry,
            new PhotonResultIntermediateBuilder(), new PhotonPipelineResultIntermediate());
        heartbeatEntry = cameraTable.getIntegerTopic("heartbeat").subscribe(-1);
        cameraIntrinsicsSubscriber =
            cameraTable.getDoubleArrayTopic("cameraIntrinsics").subscribe(null);
        cameraDistortionSubscriber =
            cameraTable.getDoubleArrayTopic("cameraDistortion").subscribe(null);
        versionEntry = photonvision_root_table.getStringTopic("version").subscribe("");

        HAL.report(edu.wpi.first.hal.FRCNetComm.tResourceType.kResourceType_PhotonCamera,
            InstanceCount);
        InstanceCount++;
    }

    /**
     * Constructs a PhotonReal from the name of the camera.
     *
     * @param cameraName The nickname of the camera (found in the PhotonVision UI).
     */
    public PhotonReal(String cameraName, String cameraIP) {
        this(NetworkTableInstance.getDefault(), cameraName, cameraIP);
    }

    private static final double[] EMPTY = new double[0];

    @Override
    public void updateInputs(PhotonInputs inputs) {
        super.updateInputs(inputs);
        var result = resultSubscriber.get();
        result.result.setTimestampSeconds((resultSubscriber.subscriber.getLastChange() / 1e6)
            - result.result.getLatencyMillis() / 1e3);
        inputs.rawBytes = result.rawBytes;
        inputs.result = result.result;

        String versionString = versionEntry.get("");
        inputs.versionString = versionString;

        var curHeartbeat = heartbeatEntry.get();
        var now = Timer.getFPGATimestamp();

        if (curHeartbeat != prevHeartbeatValue) {
            prevHeartbeatChangeTime = now;
            prevHeartbeatValue = curHeartbeat;
        }

        inputs.timeSinceLastHeartbeat = (now - prevHeartbeatChangeTime);

        inputs.cameraMatrix = cameraIntrinsicsSubscriber.get(EMPTY);
        inputs.distCoeffs = cameraDistortionSubscriber.get(EMPTY);
    }

    @Override
    public void close() throws Exception {
        resultSubscriber.close();
        heartbeatEntry.close();
        cameraIntrinsicsSubscriber.close();
        cameraDistortionSubscriber.close();
    }

}
