package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/** To do:
 * Bare bones vision control ( no checks, no ambiguity)
 * pull request after
 * Then start working on ambiguity if necessary/if have time
 * 
 * 
 */

/** Optional Calling, if the target is not found, this subsystem will not be called */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera instanceCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final String cameraName;
    private Optional<VisionData> lastVisionResult;

    private Matrix<N3, N1> PHOTON_CAM_VISION_TRUST = VecBuilder.fill(0.5, 0.5, 1);   
    private Matrix<N3, N1> SINGLE_TAG_STANDARD_DEVIATION = VecBuilder.fill(4, 4, 8);
    private Matrix<N3, N1> MULTPLE_TAG_STANDARD_DEVIATION = VecBuilder.fill(0.5, 0.5, 1);

    public VisionSubsystem(Transform3d cameraRelativeToRobot, String cameraName) {
        photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), 
            cameraRelativeToRobot);
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

    private Optional<VisionData> getVisionResult() {
        Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

        for (var result : instanceCamera.getAllUnreadResults()) {
            lastEstimatedPose = photonPoseEstimator.estimateAverageBestTargetsPose(result);
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
                PHOTON_CAM_VISION_TRUST // we should calculate this the same way photonVision does
                // in their example code
                ));
    }

    /** Calculates new standard deviations based on the number of tags, estimation strategy, and distance from tags
     * @param visionEstimation estimated position from this is used to guess standard deviations
     * @param targets Includes every target in the camera frame
     */
    public void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> visionEstimation, List<PhotonTrackedTarget> targets) {
        if (visionEstimation.isEmpty()) { // No position estimated, default to the standard deviation value
            PHOTON_CAM_VISION_TRUST = SINGLE_TAG_STANDARD_DEVIATION;
        } else { // Position estimated
            var estimateStandardDeviation = SINGLE_TAG_STANDARD_DEVIATION;
            int numTags = 0;
            double averageDistance = 0;

            // find number of tags found, get estimated distance metric
            for (var tag : targets) {
                /** Finds the tag position by searching through every tag on the field, gets the position of the tag
                 * found based on its id.
                 */
                var tagPosition = photonPoseEstimator.getFieldTags().getTagPose(tag.getFiducialId());
                if (tagPosition.isEmpty()) continue; // if no tags are found, nothing happens
                numTags++;
                averageDistance += tagPosition
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(visionEstimation
                                .get()
                                .estimatedPose
                                .toPose2d()
                                .getTranslation()); // edit the estimatedPose
            }
            if (visionEstimation.isEmpty()) { // No position estimated, default to the standard deviation value
                PHOTON_CAM_VISION_TRUST = SINGLE_TAG_STANDARD_DEVIATION;
            } else { // Position estimated
                averageDistance /= numTags;
                if (numTags > 1) {
                    estimateStandardDeviation = MULTPLE_TAG_STANDARD_DEVIATION;
                }
                if (numTags == 1 && averageDistance > 4) {
                    estimateStandardDeviation = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estimateStandardDeviation =
                            estimateStandardDeviation.times(1 + (averageDistance * averageDistance / 30));
                    PHOTON_CAM_VISION_TRUST = estimateStandardDeviation;
                }
            }
        }
    }

}
