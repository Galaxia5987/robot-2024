package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public record VisionResult(
        Pose3d estimatedRobotPose,
        double timestamp,
        double[] distanceToTargets,
        boolean useForEstimation) {}
