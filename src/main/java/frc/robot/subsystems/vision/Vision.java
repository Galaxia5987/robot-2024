package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

    private static Vision INSTANCE = null;

    private final frc.robot.subsystems.vision.VisionModule[] modules;
    private final VisionResult[] results;

    private Vision(frc.robot.subsystems.vision.VisionModule... modules) {
        this.modules = modules;
        results = new VisionResult[modules.length];
    }

    public static Vision getInstance() {
        return INSTANCE;
    }

    public static void initialize(frc.robot.subsystems.vision.VisionModule... modules) {
        INSTANCE = new Vision(modules);
    }

    public VisionResult[] getResults() {
        return results;
    }

    public List<VisionIO.ScoreParameters> getScoreParameters() {
        List<VisionIO.ScoreParameters> params = new ArrayList<>();
        Arrays.stream(modules).forEach((module) -> params.addAll(module.getScoreParameters()));
        return params;
    }

    public OptionalDouble getYawToNote() {
        for (VisionModule module : modules) {
            if (module.getYawToNote().isPresent()) {
                return module.getYawToNote();
            }
        }
        return OptionalDouble.empty();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            for (int j = 0; j < modules[i].ios.length; j++) {
                module.ios[j].updateInputs(module.inputs[j]);
                Logger.processInputs(module.ios[j].getName(), module.inputs[j]);
                results[i] = module.ios[j].getLatestResult();
            }
        }
    }
}
