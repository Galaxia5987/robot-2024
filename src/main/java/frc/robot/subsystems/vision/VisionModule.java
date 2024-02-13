package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.lib.Utils;
import java.util.Arrays;

public class VisionModule {
    public final VisionIO[] ios;
    public final VisionInputsAutoLogged[] inputs;
    public final Transform3d[] robotToCams;

    public VisionModule(VisionIO... ios) {
        this.ios = ios;
        this.inputs = new VisionInputsAutoLogged[ios.length];
        Arrays.setAll(inputs, (i) -> new VisionInputsAutoLogged());
        robotToCams =
                Arrays.stream(ios)
                        .map(VisionIO::getCameraToRobot)
                        .toList()
                        .toArray(new Transform3d[0]);
    }

    public double getAverageAmbiguity() {
        return Utils.averageAmbiguity(
                Arrays.stream(inputs).map((inputs) -> inputs.averageAmbiguity).toList());
    }
}
