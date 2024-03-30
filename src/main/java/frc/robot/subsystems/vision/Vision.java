package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private static Vision INSTANCE = null;

    private final frc.robot.subsystems.vision.VisionModule[] modules;
    private final List<VisionResult> results;

    private Vision(frc.robot.subsystems.vision.VisionModule... modules) {
        this.modules = modules;
        results = new ArrayList<>();
    }

    public static Vision getInstance() {
        return INSTANCE;
    }

    public static void initialize(frc.robot.subsystems.vision.VisionModule... modules) {
        INSTANCE = new Vision(modules);
    }

    public VisionResult[] getResults() {
        return results.toArray(new VisionResult[0]);
    }

    public List<VisionIO.ScoreParameters> getScoreParameters() {
        List<VisionIO.ScoreParameters> params = new ArrayList<>();
        Arrays.stream(modules).forEach((module) -> params.addAll(module.getScoreParameters()));
        return params;
    }

    public Optional<Rotation2d> getYawToNote() {
        for (VisionModule module : modules) {
            if (module.getYawToNote().isPresent()) {
                return module.getYawToNote();
            }
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        results.clear();
        for (VisionModule module : modules) {
            module.updateEstimationResults();
            for (int j = 0; j < module.ios.length; j++) {
                module.ios[j].updateInputs(module.inputs[j]);
                Logger.processInputs(module.ios[j].getName(), module.inputs[j]);
                results.addAll(Set.of(module.results));
            }
        }
    }
}
