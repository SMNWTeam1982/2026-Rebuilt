package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Tunables.FieldTunables;
import frc.robot.Subsystems.Drive.DriveSubsystem;

public final class HubCommands {
    public static Command moveAlongHubArc(DriveSubsystem drive, Supplier<Double> orbitDirection, boolean blueHub){
        Translation2d hubCenter;

        if (blueHub) {
            hubCenter = FieldMeasurements.BLUE_HUB_CENTER;
        }else{
            hubCenter = FieldMeasurements.RED_HUB_CENTER;
        }

        return drive.orbitPoint(orbitDirection, hubCenter, FieldTunables.HUB_SCORING_DISTANCE);
    }
}