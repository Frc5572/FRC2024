package frc.robot.subsystems.vision;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.apache.http.HttpEntity;
import org.apache.http.client.methods.CloseableHttpResponse;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.entity.mime.MultipartEntityBuilder;
import org.apache.http.entity.mime.content.FileBody;
import org.apache.http.impl.client.CloseableHttpClient;
import org.apache.http.impl.client.HttpClients;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.TimeUtils;

public class AprilTagVisionIOReal implements AprilTagVisionIO {
    protected final PhotonCamera[] cameras;
    HashMap<String, AtomicBoolean> isPhotonOk = new HashMap<String, AtomicBoolean>();

    public AprilTagVisionIOReal(List<PhotonCameraProperties> cameraProperties) {
        if (cameraProperties.size() > 16)
            throw new IllegalArgumentException("max supported camera count is 16");
        cameras = new PhotonCamera[cameraProperties.size()];

        for (int i = 0; i < cameraProperties.size(); i++) {
            var cameraName = cameraProperties.get(i).name;
            var ipAddress = cameraProperties.get(i).ipAddress;
            cameras[i] = new PhotonCamera(cameraName);
            isPhotonOk.put(cameraName, new AtomicBoolean(false));
            new Thread(() -> {
                Timer timer = new Timer();
                while (true) {
                    if (timer.advanceIfElapsed(5.0) && !isPhotonOk.get(cameraName).get()) {
                        try {
                            uploadSettings(cameraName, ipAddress + ":5800",
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

        // PortForwarder.add(5800, "photonvision", 5800);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        if (inputs.camerasAmount != cameras.length)
            throw new IllegalStateException("inputs camera amount (" + inputs.camerasAmount
                + ") does not match actual cameras amount");

        for (int i = 0; i < cameras.length; i++)
            if (cameras[i].isConnected())
                inputs.camerasInputs[i].fromPhotonPipeLine(cameras[i].getLatestResult(),
                    cameras[i].isConnected());
            else
                inputs.camerasInputs[i].clear();
        inputs.inputsFetchedRealTimeStampSeconds = TimeUtils.getRealTimeSeconds();
    }

    @Override
    public void close() {
        for (PhotonCamera camera : cameras)
            camera.close();
    }

    public boolean uploadSettings(String name, String ip, File file) throws IOException {
        try (final CloseableHttpClient httpClient = HttpClients.createDefault()) {
            HttpPost postReq = new HttpPost("http://" + ip + "/api/settings");
            HttpEntity entity =
                MultipartEntityBuilder.create().addPart("data", new FileBody(file)).build();
            postReq.setEntity(entity);
            try (CloseableHttpResponse response = httpClient.execute(postReq)) {
                SmartDashboard.putString("uploadSettings/" + name + "/status",
                    response.getStatusLine().getStatusCode() + ": "
                        + response.getStatusLine().getReasonPhrase());
                var ent = response.getEntity();
                if (ent != null) {
                    try (InputStream stream = ent.getContent()) {
                        String text = new String(stream.readAllBytes(), StandardCharsets.UTF_8);
                        SmartDashboard.putString("uploadSettings/" + name + "/content", text);
                    }
                } else {
                    SmartDashboard.putString("uploadSettings/" + name + "/content", "null");
                }
                return response.getStatusLine().getStatusCode() == 200;
            }
        }
    }
}
