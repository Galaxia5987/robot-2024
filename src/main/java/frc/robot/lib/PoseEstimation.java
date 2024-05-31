package frc.robot.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Arrays;
import java.util.Optional;
import java.util.stream.DoubleStream;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseEstimation {
    private static PoseEstimation INSTANCE = null;

    private final SwerveDrive swerveDrive;

    private Translation2d speakerPose;

    private PoseEstimation() {
        swerveDrive = SwerveDrive.getInstance();
    }

    public static PoseEstimation getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PoseEstimation();
        }
        return INSTANCE;
    }

    @AutoLogOutput(key = "Robot/EstimatedRobotPose")
    public Pose2d getEstimatedPose() {
        return swerveDrive.getEstimator().getEstimatedPosition();
    }

    @AutoLogOutput(key = "Shooter/DistanceToSpeaker")
    public double getDistanceToSpeaker() {
        return getPoseRelativeToSpeaker().getNorm();
    }

    @AutoLogOutput(key = "Robot/ToSpeaker")
    public Translation2d getPoseRelativeToSpeaker() {
        Optional<DriverStation.Alliance> allianceColor = DriverStation.getAlliance();
        if (allianceColor.isPresent() && (allianceColor.get() == DriverStation.Alliance.Red)) {
            speakerPose = Constants.SPEAKER_POSE_RED;
        } else {
            speakerPose = Constants.SPEAKER_POSE_BLUE;
        }
        return speakerPose.minus(getEstimatedPose().getTranslation());
    }
}
