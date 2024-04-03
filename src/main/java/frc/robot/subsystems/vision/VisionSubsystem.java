package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera unicornLeft;
    private final PhotonCamera meowRight;
    private final PhotonPoseEstimator unicornLeftEstimator;
    private final PhotonPoseEstimator meowRightEstimator;

    public VisionSubsystem() {
        unicornLeft = new PhotonCamera("unicornleft");
        meowRight = new PhotonCamera("meowRight");
        unicornLeftEstimator = new PhotonPoseEstimator(null, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null);
        meowRightEstimator = new PhotonPoseEstimator(null, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null);
    }

}
