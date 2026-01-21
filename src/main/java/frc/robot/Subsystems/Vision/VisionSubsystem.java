package frc.robot.Subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private Optional<VisionData> lastVisionResult;

    public VisionSubsystem(){}

    public Optional<VisionData> getLastVisionResult(){
        return lastVisionResult;
    }

    /** fetches the data from the limelight and then updates the internal variable */
    public Command getLatestVisionResults(){
        return run(
            () -> {}
        );
    }
}