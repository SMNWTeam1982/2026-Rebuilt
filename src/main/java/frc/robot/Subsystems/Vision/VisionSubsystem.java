package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.lang.System.Logger;

public class VisionSubsystem extends SubsystemBase {

    private Optional<VisionData> lastVisionResult;
    private boolean isDataUpToDate = false;

    public Trigger recentData = new Trigger(() -> isDataUpToDate);

    @Override
    public void Periodic() {
        var visionResult = getLatestVisionResults();
        if(visionResult.isPresent()){
            isDataUpToDate = true;
            lastVisionResult = visionResult.get();
            logPoseEstimation(lastVisionResult.pose());
        } else
    }
    
    public VisionSubsystem() {}

    public Optional<VisionData> getLastVisionResult() {
        return lastVisionResult;
    }

    /** fetches the data from the limelight and then updates the internal variable */
    public Command getLatestVisionResults() {
        return run(() -> {});
    }

    public Command getDistance(){
        
    }
}
