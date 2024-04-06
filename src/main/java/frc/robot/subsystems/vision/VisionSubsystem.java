package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera unicornLeft;
    private final PhotonCamera meowRight;
    private final PhotonPoseEstimator unicornLeftEstimator;
    private final PhotonPoseEstimator meowRightEstimator;
    private final Transform3d unicornLeftTransform;
    private final Transform3d meowRightTransform;
    private double lastEstTimestamp = 0;
    // CAMERAS WERE NAMED LOOKING AT IT FROM THE INTAKE SIDE. Made a mistake, too lazy to change it
    // NCML - No case meow left. This cam is on the RSL side.
    // CUR - Case unicorn right. This cam is on the radio side.
    public VisionSubsystem() {
        unicornLeft = new PhotonCamera("unicornLeft");
        meowRight = new PhotonCamera("meowRight");
        unicornLeftTransform = new Transform3d(new Translation3d(Units.inchesToMeters(-11.302), Units.inchesToMeters(-12.747), Units.inchesToMeters(8.613)), new Rotation3d(0, Units.degreesToRadians(61.875), Units.degreesToRadians(210)));
        meowRightTransform = new Transform3d(new Translation3d(Units.inchesToMeters(-11.302), Units.inchesToMeters(12.747), Units.inchesToMeters(8.613)), new Rotation3d(0, Units.degreesToRadians(61.875), Units.degreesToRadians(150)));
        unicornLeftEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, unicornLeft, unicornLeftTransform);
        meowRightEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, meowRight, meowRightTransform);
        unicornLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        meowRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    // Return left camera
    public PhotonCamera getUnicornLeft() {
        return unicornLeft;
    }
    // Return right camera
    public PhotonCamera getMeowRight() {
        return meowRight;
    }
    // Return left camera pose estimator
    public PhotonPoseEstimator getUnicornLeftEst() {
        return unicornLeftEstimator;
    }
    // Return right camera pose estimator
    public PhotonPoseEstimator getMeowRightEst() {
        return meowRightEstimator;
    }
    // Return left camera results
    public PhotonPipelineResult getLatestResult (PhotonCamera camera) {
        return camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoses(Supplier<PhotonCamera> camera, Supplier<PhotonPoseEstimator> poseEstimator) {
        var visionEst = poseEstimator.get().update();
        double latestTimestamp = camera.get().getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, Supplier<PhotonPoseEstimator> poseEstimator, Supplier<PhotonCamera> camera) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = camera.get().getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = poseEstimator.get().getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

}
