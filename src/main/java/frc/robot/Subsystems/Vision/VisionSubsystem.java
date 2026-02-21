package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANBus.VisionConstants;
import frc.robot.Constants.Tunables.VisionTunables;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * what does the array of getAllCameraResults() look like?
 * is the poses that they put out similar?
 * are the timestamps and visible targets similar?
 * what is the difference between returning the last result in the array vs the "best" one
 */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera instanceCamera = new PhotonCamera(VisionConstants.limeLightCameraName);
    private final PhotonPoseEstimator photonPoseEstimator = 
        new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                VisionTunables.PHOTON_CAM_RELATIVE_TO_ROBOT);
    
    private Optional<VisionData> lastVisionResult = Optional.empty();
    private int tagsVisible = 0;

    public VisionSubsystem() {
        setDefaultCommand(pollVisionData());
    }

    @Override
    public void periodic(){
        Logger.recordOutput("Vision/tags visible", tagsVisible);
    }

    public Command pollVisionData(){
        return run(
            () -> {
                Optional<EstimatedRobotPose> bestResult = Optional.empty();

                // look at all results from the camera
                for (PhotonPipelineResult result : instanceCamera.getAllUnreadResults()){
                    // get the multitarget tag pose because its the most accurate
                    Optional<EstimatedRobotPose> estimate = photonPoseEstimator.estimateCoprocMultiTagPose(result);
                    if (estimate.isEmpty()) { 
                        // if we fail to get the multi tag pose then get the lowest ambiguity pose
                        estimate = photonPoseEstimator.estimateLowestAmbiguityPose(result);
                    }

                    if (bestResult.isEmpty()){ // replace the best result with whatever if the best result is empty
                        bestResult = estimate;
                    }else if (estimate.isPresent()){ // if we need to compare the two
                        int oldResultTargets = bestResult.get().targetsUsed.size();
                        int newResultTargets = estimate.get().targetsUsed.size();

                        if (newResultTargets > oldResultTargets){ 
                            // pick the one with the most targets
                            bestResult = estimate;
                        }else if (newResultTargets == oldResultTargets){ 
                            // if the same number of targets then pick the one with the most recent timestamp
                            if (estimate.get().timestampSeconds > bestResult.get().timestampSeconds){
                                bestResult = estimate;
                            }
                        }
                    }
                }

                bestResult.ifPresentOrElse(
                    poseEstimate -> {
                        Pose2d robotPose = poseEstimate.estimatedPose.toPose2d();
                        double timestamp = poseEstimate.timestampSeconds;
                        tagsVisible = poseEstimate.targetsUsed.size();
                        // reduce the deviation if we see more targets
                        Matrix<N3, N1> standardDeviation = VisionTunables.PHOTON_CAM_VISION_TRUST.div(tagsVisible);

                        lastVisionResult = Optional.of(new VisionData(robotPose, timestamp, standardDeviation));
                    },
                    () -> {
                        tagsVisible = 0;
                        lastVisionResult = Optional.empty();
                    }
                );
            }
        );
    }

    public Optional<VisionData> getLastVisionResult() {
        return lastVisionResult;
    }
}
