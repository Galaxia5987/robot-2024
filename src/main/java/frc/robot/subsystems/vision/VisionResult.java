package frc.robot.subsystems.vision;

import lombok.Getter;
import lombok.Setter;
import org.photonvision.EstimatedRobotPose;

public class VisionResult {
    @Setter @Getter private EstimatedRobotPose estimatedRobotPose;
    @Setter @Getter private boolean useForEstimation;

    public VisionResult(EstimatedRobotPose pose, boolean useForEstimation) {
        this.estimatedRobotPose = pose;
        this.useForEstimation = useForEstimation;
    }
}
