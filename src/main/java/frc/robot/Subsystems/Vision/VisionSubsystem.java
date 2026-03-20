package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.VisionConstants;
import frc.robot.Constants.Measured.VisionMeasurements;
import frc.robot.Constants.Tunables.VisionTunables;

import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** To do:
 * Bare bones vision control ( no checks, no ambiguity)
 * pull request after
 * Then start working on ambiguity if necessary/if have time
 */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera instanceCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Optional<VisionData> lastVisionResult = Optional.empty();

    public VisionSubsystem() {
        photonPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                VisionMeasurements.PHOTON_CAM_RELATIVE_TO_ROBOT);
        instanceCamera = new PhotonCamera(VisionConstants.limeLightCameraName);
    }

    @Override
    public void periodic(){
        lastVisionResult = getVisionResult();
    }

    public Optional<VisionData> getLastVisionResult() {
        return lastVisionResult;
    }

    public String getName() {
        return VisionConstants.limeLightCameraName;
    }

    /** When called, this gets the last Estimated Position of the Robot and estimates the average position of the targets.
     *  If there are no more targets it will return Empty.
     *  Then it gets the last estimated pose before replacing VisionData.
     */
    private Optional<VisionData> getVisionResult() {
        List<PhotonPipelineResult> cameraResults = instanceCamera.getAllUnreadResults();

        if (cameraResults.isEmpty()){
            // return early if no data from camera
            return Optional.empty();
        }


        Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

        for (var result : cameraResults) {
            // Estimates the average position of the targets based on the targets last position
            lastEstimatedPose = photonPoseEstimator.estimateCoprocMultiTagPose(result);

            if (lastEstimatedPose.isEmpty()) {
                /** If the last estimated position is empty the last estimated pose will be used to estimate the position
                 * with the lowest ambiguity.
                 */
                lastEstimatedPose = photonPoseEstimator.estimateLowestAmbiguityPose(result);
            }
        }

        // lastEstimatedPose will be empty if the camera result has no targets
        // only update the visible target logging if we have a camera result to log
        if (lastEstimatedPose.isEmpty()) {
            Logger.recordOutput("Vision/visible targets", 0);
            return Optional.empty();
        }

        EstimatedRobotPose estimatedPose = lastEstimatedPose.get();

        int visibleTags = estimatedPose.targetsUsed.size();

        Logger.recordOutput("Vision/visible targets", visibleTags);

        return Optional.of(new VisionData(
                estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds,
                VisionTunables.STANDARD_DEVIATIONS.div(visibleTags) // more trust with more tags
                ));
    }
}
