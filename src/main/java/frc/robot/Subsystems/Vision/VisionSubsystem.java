package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tunables.VisionTunables;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** To do:
 * Bare bones vision control ( no checks, no ambiguity)
 * pull request after
 * Then start working on ambiguity if necessary/if have time
 */

/** Optional Calling, if the target is not found, this subsystem will not be called */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera instanceCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final String cameraName;
    private Optional<VisionData> lastVisionResult;

    /**
     * @param cameraName = VisionConstants.limeLightCameraName
     */
    public VisionSubsystem(Transform3d cameraRelativeToRobot, String cameraName) {
        photonPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), cameraRelativeToRobot);
        instanceCamera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        setDefaultCommand(pollVisionData());
    }

    public Command pollVisionData() { // Factory
        return run(() -> {
            lastVisionResult = getVisionResult();
        });
    }

    public Optional<VisionData> getLastVisionResult() {
        return lastVisionResult;
    }

    public String getName() {
        return cameraName;
    }

    /** When called, this gets the last Estimated Position of the Robot and estimates the average position of the targets.
     *  If there are no more targets it will return Empty.
     *  Then it gets the last estimated pose before replacing VisionData.
     */
    private Optional<VisionData> getVisionResult() {
        Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

        for (var result : instanceCamera.getAllUnreadResults()) {
            // Estimates the average position of the targets based on the targets last position
            lastEstimatedPose = photonPoseEstimator.estimateCoprocMultiTagPose(result);
            if (lastEstimatedPose.isEmpty()) {
                /** If the last estimated position is empty the last estimated pose will be used to estimate the position
                 * with the lowest ambiguity.
                 */
                lastEstimatedPose = photonPoseEstimator.estimateLowestAmbiguityPose(result);
            }
        }
        // if getAllUnreadResults() is empty then lastEstimatedPose will be Optional.empty()
        // also accounts for results that have data but are surrounded by results without data
        if (lastEstimatedPose.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose estimatedPose = lastEstimatedPose.get();

        return Optional.of(new VisionData(
                estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds,
                VisionTunables.PHOTON_CAM_VISION_TRUST // we should calculate this the same way photonVision does
                // in their example code
                ));
    }
}
