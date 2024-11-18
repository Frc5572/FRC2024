package frc.robot.subsystems.vision;

import java.util.function.Function;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.StateEstimator;
import frc.robot.Constants.Camera;
import frc.robot.subsystems.vision.CameraIO.CameraInputs;

public class Vision extends SubsystemBase {

    private final StateEstimator estimator;
    private final CameraIO[] cameraIOs;
    private final CameraConstants[] constants;
    private final CameraInputs[] inputs;

    public Vision(Function<CameraConstants, CameraIO> cameraFunction, StateEstimator estimator) {
        super("Vision");
        this.estimator = estimator;
        this.cameraIOs = Stream.of(Camera.CONSTANTS).map(cameraFunction).toArray(CameraIO[]::new);
        this.constants = Camera.CONSTANTS;
        this.inputs = Stream.of(Camera.CONSTANTS).map((_k) -> new CameraInputs())
            .toArray(CameraInputs[]::new);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < cameraIOs.length; i++) {
            cameraIOs[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + constants[i].cameraName(), inputs[i]);

            estimator.visionUpdate(inputs[i].result, constants[i]);
        }
    }

}
