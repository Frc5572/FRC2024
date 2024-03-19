package frc.lib.util.watson;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;

public class WatsonCamera extends WatsonIO {

    private final NetworkTable cameraTable;
    private RawSubscriber rawBytesEntry;

    public WatsonCamera(String name) {
        super(name);
        cameraTable = NetworkTableInstance.getDefault().getTable("watson");
        rawBytesEntry = cameraTable.getRawTopic(name).subscribe("rawBytes", new byte[] {},
            PubSubOption.periodic(0.02), PubSubOption.sendAll(true));
    }

    @Override
    public void updateInputs(WatsonInputs input) {
        super.updateInputs(input);
        byte[] data = rawBytesEntry.get(new byte[] {});
        input.rawBytes = data;
        input.deserializeResult();
    }

}
