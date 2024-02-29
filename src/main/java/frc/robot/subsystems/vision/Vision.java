package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Vision extends SubsystemBase {

    private static Vision INSTANCE = null;

    private final frc.robot.subsystems.vision.VisionModule[] modules;
    private final EstimatedRobotPose[] results;

    private Vision(frc.robot.subsystems.vision.VisionModule... modules) {
        this.modules = modules;
        results = new EstimatedRobotPose[modules.length];
    }

    public static Vision getInstance() {
        return INSTANCE;
    }

    public static void initialize(frc.robot.subsystems.vision.VisionModule... modules) {
        INSTANCE = new Vision(modules);
    }

    public EstimatedRobotPose[] getResults() {
        return results;
    }

    public List<VisionIO.ScoreParameters> getScoreParameters() {
        List<VisionIO.ScoreParameters> params = new ArrayList<>();
        Arrays.stream(modules).forEach((module) -> params.addAll(module.getScoreParameters()));
        return params;
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
