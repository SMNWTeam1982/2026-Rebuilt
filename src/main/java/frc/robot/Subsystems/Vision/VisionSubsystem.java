package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANBus.VisionConstants;
import frc.robot.Constants.Measured.VisionMeasurements;
import frc.robot.Constants.Tunables.VisionTunables;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * manages the Limelight that is on the robot fetching poses and logging data from it
 * <p> when LED mode is enabled the LEDs on the limelight will turn on when it sees an april tag in order to help with field set up
 * <p> LED mode is enabled by default
 */
public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera instanceCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Optional<VisionData> lastVisionResult = Optional.empty();

    public final Trigger hasVisionResult = new Trigger(lastVisionResult::isPresent);

    @AutoLogOutput(key = "Vision/LED mode enabled")
    private boolean useLEDs = true;

    private final Alert gotCameraResult = new Alert("recieved camera result from limelight", AlertType.kInfo);

    public VisionSubsystem() {
        photonPoseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                VisionMeasurements.PHOTON_CAM_RELATIVE_TO_ROBOT);
        instanceCamera = new PhotonCamera(VisionConstants.limeLightCameraName);

        hasVisionResult.debounce(1.0, DebounceType.kFalling).onTrue(turnLEDsOn()).onFalse(turnLEDsOff());
    }

    @Override
    public void periodic() {
        lastVisionResult = getVisionResult();
    }

    public Optional<VisionData> getLastVisionResult() {
        return lastVisionResult;
    }

    public String getName() {
        return VisionConstants.limeLightCameraName;
    }

    public Command turnLEDsOn(){
        return runOnce(
            () -> instanceCamera.setLED(VisionLEDMode.kOn)
        ).onlyIf(() -> useLEDs).ignoringDisable(true);
    }

    public Command turnLEDsOff(){
        return runOnce(
            () -> instanceCamera.setLED(VisionLEDMode.kOff)
        ).onlyIf(() -> useLEDs).ignoringDisable(true);
    }

    /** when LED mode is enabled, the LEDs on the limelight will turn on when it sees an april tag */
    public Command activateLEDMode() {
        return runOnce(() -> {
                    useLEDs = true;
                })
                .ignoringDisable(true);
    }

    /** when LED mode is disabled the limelight's LEDs will turn off and stay off */
    public Command deactivateLEDMode() {
        return runOnce(() -> {
                    useLEDs = false;
                })
                .ignoringDisable(true);
    }

    /** When called, this gets the last Estimated Position of the Robot and estimates the average position of the targets.
     *  If there are no more targets it will return Empty.
     *  Then it gets the last estimated pose before replacing VisionData.
     */
    private Optional<VisionData> getVisionResult() {
        List<PhotonPipelineResult> cameraResults = instanceCamera.getAllUnreadResults();

        if (cameraResults.isEmpty()) {
            // return early if no data from camera
            gotCameraResult.set(false);
            return Optional.empty();
        } else {
            gotCameraResult.set(true);
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

        Pose3d estimated3dRobotPose = estimatedPose.estimatedPose;

        Logger.recordOutput("Vision/3d pose estimate", estimated3dRobotPose);

        Pose2d estimated2dRobotPose = estimated3dRobotPose.toPose2d();

        Logger.recordOutput("Vision/2d pose estimate", estimated2dRobotPose);

        Logger.recordOutput("Vision/strategy", estimatedPose.strategy.toString());

        return Optional.of(new VisionData(
                estimated2dRobotPose,
                estimatedPose.timestampSeconds,
                VisionTunables.STANDARD_DEVIATIONS.div(visibleTags) // more trust with more tags
                ));
    }
}
