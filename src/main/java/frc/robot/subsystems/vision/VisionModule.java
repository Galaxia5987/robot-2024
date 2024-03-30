package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

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

    public List<VisionIO.ScoreParameters> getScoreParameters() {
        return Arrays.stream(inputs)
                .filter((inputs) -> inputs.hasScoreParams)
                .map(
                        (inputs) ->
                                new VisionIO.ScoreParameters(
                                        inputs.toSpeaker,
                                        inputs.seesSpeaker
                                                ? Optional.of(inputs.yawToSpeaker)
                                                : Optional.empty()))
                .toList();
    }

    public Optional<Rotation2d> getYawToNote() {
        for (var inputs : inputs) {
            if (inputs.hasNote) {
                return Optional.of(inputs.yawNote);
            }
        }
        return Optional.empty();
    }
}
