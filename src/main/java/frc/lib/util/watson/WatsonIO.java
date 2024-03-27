package frc.lib.util.watson;

import java.nio.ByteBuffer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public abstract class WatsonIO {

    /**
     * Inputs from PhotonVision.
     */
    public static class WatsonInputs implements LoggableInputs {

        public int timeMicros;
        public byte[] rawBytes;
        public boolean isMultiTag;
        public int[] seenTags;
        public Pose3d result;
        public double reprojectionError;
        public Pose3d altResult;
        public double altReprojectionError;
        public String name;

        /**
         * Default Constructor.
         */
        public WatsonInputs() {
            rawBytes = new byte[] {};
            name = "";
            result = null;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("rawBytes", rawBytes);
            table.put("name", name);
        }

        @Override
        public void fromLog(LogTable table) {
            this.rawBytes = table.get("rawBytes", rawBytes);
            this.deserializeResult();
            this.name = table.get("name", name);
        }

        public void deserializeResult() {
            if (this.rawBytes.length < 20) {
                this.result = null;
                this.seenTags = new int[0];
                return;
            }
            ByteBuffer buf = ByteBuffer.wrap(this.rawBytes);
            buf.rewind();
            this.timeMicros = buf.getInt();
            int len = buf.getInt();
            this.seenTags = new int[len];
            for (int i = 0; i < len; i++) {
                seenTags[i] = buf.getInt();
            }
            if (buf.get() == 1) {
                this.isMultiTag = false;
            } else {
                this.isMultiTag = true;
            }
            double x = buf.getDouble();
            double y = buf.getDouble();
            double z = buf.getDouble();
            Translation3d t0 = new Translation3d(x, y, z);
            double rw = buf.getDouble();
            double rx = buf.getDouble();
            double ry = buf.getDouble();
            double rz = buf.getDouble();
            Rotation3d r0 = new Rotation3d(new Quaternion(rw, rx, ry, rz));
            this.result = new Pose3d(t0, r0);
            this.reprojectionError = buf.getDouble();
            if (!this.isMultiTag) {
                double x2 = buf.getDouble();
                double y2 = buf.getDouble();
                double z2 = buf.getDouble();
                Translation3d t1 = new Translation3d(x2, y2, z2);
                double rw2 = buf.getDouble();
                double rx2 = buf.getDouble();
                double ry2 = buf.getDouble();
                double rz2 = buf.getDouble();
                Rotation3d r1 = new Rotation3d(new Quaternion(rw2, rx2, ry2, rz2));
                this.altResult = new Pose3d(t1, r1);
                this.altReprojectionError = buf.getDouble();
            }
        }

    }

    private final String name;

    public WatsonIO(String name) {
        this.name = name;
    }

    public void updateInputs(WatsonInputs input) {
        input.name = name;
    }

}
