package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionIOReal implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final PhotonPoseEstimator simpleEstimator;
    private final Transform3d robotToCamera;
    private final AprilTagFieldLayout field;
    private final boolean calculateScoreParams;
    private VisionResult result =
            new VisionResult(
                    new EstimatedRobotPose(
                            new Pose3d(),
                            0,
                            new ArrayList<>(),
                            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                    false);
    private VisionResult lastResult =
            new VisionResult(
                    new EstimatedRobotPose(
                            new Pose3d(),
                            0,
                            new ArrayList<>(),
                            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
                    false);
    private Optional<ScoreParameters> scoreParameters = Optional.empty();
    private final LinearFilter xFilter = LinearFilter.movingAverage(20);
    private final LinearFilter yFilter = LinearFilter.movingAverage(20);
    private final LinearFilter yawFilter = LinearFilter.movingAverage(20);

    public PhotonVisionIOReal(
            PhotonCamera camera,
            Transform3d robotToCamera,
            AprilTagFieldLayout field,
            boolean calculateScoreParams) {
        this.camera = camera;
        this.robotToCamera = robotToCamera;
        this.field = field;
        this.calculateScoreParams = calculateScoreParams;
        camera.setPipelineIndex(0);
        estimator =
                new PhotonPoseEstimator(
                        field,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        camera,
                        robotToCamera);

        simpleEstimator =
                new PhotonPoseEstimator(
                        field,
                        PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
                        camera,
                        robotToCamera);
    }

    @Override
    public void setPipeLine(int pipeLineIndex) {
        camera.setPipelineIndex(pipeLineIndex);
    }

    @Override
    public Optional<ScoreParameters> getScoreParameters() {
        return scoreParameters;
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.isConnected = camera.isConnected();

        var latestResult = camera.getLatestResult();

        if (calculateScoreParams) {
            var robotPose = simpleEstimator.update(latestResult);
            if (robotPose.isPresent()) {
                var toSpeaker =
                        robotPose
                                .get()
                                .estimatedPose
                                .getTranslation()
                                .toTranslation2d()
                                .minus(VisionConstants.getSpeakerPose());
                toSpeaker =
                        new Translation2d(
                                xFilter.calculate(toSpeaker.getX()),
                                yFilter.calculate(toSpeaker.getY()));
                inputs.distanceToSpeaker = toSpeaker.getNorm();
                //                var centerTag =
                //                        latestResult.getTargets().stream()
                //                                .filter(
                //                                        (target) ->
                //                                                target.getFiducialId()
                //                                                        ==
                // VisionConstants.getSpeakerTag1())
                //                                .toList();
                //                if (centerTag.size() != 0) {
                //                    var translation =
                //                            centerTag
                //                                    .get(0)
                //                                    .getBestCameraToTarget()
                //                                    .plus(new Transform3d(0, -0.341, 0, new
                // Rotation3d()))
                //                                    .getTranslation()
                //                                    .toTranslation2d();
                //                    System.out.println(translation.getY());
                //                    scoreParameters =
                //                            Optional.of(
                //                                    new ScoreParameters(
                //                                            inputs.distanceToSpeaker,
                //                                            new Rotation2d(translation.getX(),
                // translation.getY())
                //                                                    .unaryMinus()));
                //                } else {
                //                    scoreParameters = Optional.empty();
                //                }
                var centerTag =
                        latestResult.getTargets().stream()
                                .filter(
                                        (target) ->
                                                target.getFiducialId()
                                                        == VisionConstants.getSpeakerTag1())
                                .toList();
                if (centerTag.size() != 0) {
                    var yaw = centerTag.get(0).getYaw();
                    scoreParameters =
                            Optional.of(
                                    new ScoreParameters(
                                            inputs.distanceToSpeaker, Rotation2d.fromDegrees(yaw)));
                } else {
                    scoreParameters = Optional.empty(); // TODO: make the rotation gyro based
                }
            } else {
                scoreParameters = Optional.empty();
            }
        }

        var estimatedPose = estimator.update(latestResult);
        if (estimatedPose.isPresent()) {
            inputs.poseFieldOriented = estimatedPose.get().estimatedPose;

            result = new VisionResult(estimatedPose.get(), true);
            double distanceTraveled =
                    result.getEstimatedRobotPose()
                            .estimatedPose
                            .minus(lastResult.getEstimatedRobotPose().estimatedPose)
                            .getTranslation()
                            .getNorm();
            if ((DriverStation.isEnabled() && distanceTraveled > 0.3)
                    || (result.getEstimatedRobotPose().estimatedPose.getZ() > 0.2)
                    || (VisionConstants.outOfBounds(
                            result.getEstimatedRobotPose().estimatedPose))) {
                result.setUseForEstimation(false);
            }
        } else {
            result.setUseForEstimation(false);
        }

        lastResult = result;
    }

    @Override
    public EstimatedRobotPose getLatestResult() {
        return result.getEstimatedRobotPose();
    }

    @Override
    public Transform3d getCameraToRobot() {
        return robotToCamera;
    }

    @Override
    public String getName() {
        return camera.getName();
    }
}
