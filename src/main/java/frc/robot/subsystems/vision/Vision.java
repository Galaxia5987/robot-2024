package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.Utils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Vision extends SubsystemBase {

    private static Vision INSTANCE = null;

    private final VisionModule[] modules;
    private final EstimatedRobotPose[] results;

    private Vision(VisionModule... modules) {
        this.modules = modules;
        results = new EstimatedRobotPose[modules.length];
    }

    public static Vision getInstance() {
        return INSTANCE;
    }

    public static void initialize(VisionModule... modules) {
        INSTANCE = new Vision(modules);
    }

    public EstimatedRobotPose[] getResults() {
        return results;
    }

    public double getAverageAmbiguity() {
        List<Double> ambiguityList =
                Arrays.stream(modules).map(VisionModule::getAverageAmbiguity).toList();
        return Utils.averageAmbiguity(ambiguityList);
    }

    @Override
    public void periodic() {
        List<Double> totalAverageAmbiguities = new ArrayList<>();
        double totalAverageAmbiguity;
        for (int i = 0; i < modules.length; i++) {
            VisionModule module = modules[i];
            for (int j = 0; j < modules[i].ios.length; j++) {
                module.ios[j].updateInputs(module.inputs[j]);
                Logger.processInputs(module.ios[j].getName(), module.inputs[j]);
                results[i] = module.ios[j].getLatestResult();
            }
            totalAverageAmbiguities.add(module.getAverageAmbiguity());
        }
        totalAverageAmbiguity = Utils.averageAmbiguity(totalAverageAmbiguities);
        Logger.recordOutput("totalAverageAmbiguity", totalAverageAmbiguity);
    }
}
