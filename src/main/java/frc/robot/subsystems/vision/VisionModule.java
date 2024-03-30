package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class VisionModule {
    public final VisionIO[] ios;
    public final VisionInputsAutoLogged[] inputs;
    public final Transform3d[] robotToCams;
    public final VisionResult[] results;
    public final VisionResult[] lastResults;

    public VisionModule(VisionIO... ios) {
        this.ios = ios;
        this.inputs = new VisionInputsAutoLogged[ios.length];
        this.results = new VisionResult[ios.length];
        this.lastResults = new VisionResult[ios.length];
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

    public void updateEstimationResults() {
        for (int i = 0; i < inputs.length; i++) {
            boolean hasResult = inputs[i].hasNewPose;
            if (hasResult) {
                double distanceTraveled =
                        lastResults[i]
                                .estimatedRobotPose()
                                .minus(inputs[i].poseFieldOriented)
                                .getTranslation()
                                .getNorm();
                boolean ignore =
                        (DriverStation.isEnabled() && distanceTraveled > 0.2)
                                || (inputs[i].poseFieldOriented.getZ() > 0.1)
                                || (VisionConstants.outOfBounds(inputs[i].poseFieldOriented))
                                || Arrays.stream(inputs[i].distanceToTargets)
                                        .anyMatch((distance) -> distance > 5.0);
                results[i] =
                        new VisionResult(
                                inputs[i].poseFieldOriented,
                                inputs[i].timestamp,
                                inputs[i].distanceToTargets,
                                !ignore);
            } else {
                results[i] =
                        new VisionResult(
                                inputs[i].poseFieldOriented,
                                inputs[i].timestamp,
                                inputs[i].distanceToTargets,
                                false);
            }
            lastResults[i] = results[i];
        }
    }
}
