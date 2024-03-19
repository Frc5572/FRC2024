package frc.lib.util.watson;

import java.nio.ByteBuffer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class WatsonIO {

    /**
     * Inputs from PhotonVision.
     */
    public static class WatsonInputs implements LoggableInputs {

        public byte[] rawBytes;
        public boolean isMultiTag;
        public Translation2d result;
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
                return;
            }
            ByteBuffer buf = ByteBuffer.wrap(this.rawBytes);
            buf.rewind();
            int len = buf.getInt();
            for (int i = 0; i < len; i++) {
                buf.getInt();
            }
            if (buf.get() == 1) {
                this.isMultiTag = false;
            } else {
                this.isMultiTag = true;
            }
            double x = buf.getDouble();
            double y = buf.getDouble();
            this.result = new Translation2d(x, y);
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
