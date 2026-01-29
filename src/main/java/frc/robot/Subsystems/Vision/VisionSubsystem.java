package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import frc.robot.Constants.CANBus.VisionConstants;

public class VisionSubsystem  {

    private final PhotonCamera instanceCamera;

    private final PhotonPoseEstimator photonPoseEstimator;

    private final String cameraName;

    public VisionSubsystem(Transform3d cameraRelativeToRobot, String cameraName) {

        photonPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                PoseStrategy.LOWEST_AMBIGUITY,
                cameraRelativeToRobot);
        instanceCamera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
    }

    public String getName() {
        return cameraName;
    }

    private Optional<VisionData> getVisionResult() {
        Optional<EstimatedRobotPose> lastEstimatedPose = Optional.empty();

        for (var result : instanceCamera.getAllUnreadResults()) {
            lastEstimatedPose = photonPoseEstimator.update(result);
        } // if getAllUnreadResults() is empty then lastEstimatedPose will be Optional.empty()
        // this also accounts for results that have data but are surrounded by results without data

        if (lastEstimatedPose.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose estimatedPose = lastEstimatedPose.get();

        return Optional.of(new VisionData(
                estimatedPose.estimatedPose.toPose2d(),
                estimatedPose.timestampSeconds,
                VisionConstants.PHOTON_CAM_VISION_TRUST // we should calculate this the same way photonVision does
                // in their example code
                ));
    }

    
}