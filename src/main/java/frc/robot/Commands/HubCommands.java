package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Tunables.FieldTunables;
import frc.robot.Subsystems.Drive.DriveSubsystem;

import java.util.function.Supplier;

public final class HubCommands {
    public static Command moveAlongHubArc(DriveSubsystem drive, Supplier<Double> orbitDirection, boolean blueHub) {
        Translation2d hubCenter;

        if (blueHub) {
            hubCenter = FieldMeasurements.BLUE_HUB_CENTER;
        } else {
            hubCenter = FieldMeasurements.RED_HUB_CENTER;
        }

        return drive.orbitPoint(orbitDirection, hubCenter, FieldTunables.HUB_SCORING_DISTANCE);
    }

    public static Translation2d calculateScoringTarget(Translation2d robotPosition, Translation2d hubPosition, ChassisSpeeds robotVelocity){
        double distanceFromHub = robotPosition.getDistance(hubPosition);

        double flightTime = FieldMeasurements.hubDistanceToFlightTime(distanceFromHub);

        Translation2d robotTravel = new Translation2d(
            robotVelocity.vxMetersPerSecond * flightTime,
            robotVelocity.vyMetersPerSecond * flightTime
        );

        // move the hub in the opposite direction of the robots travel to simulate where it going to be when the ball lands
        return hubPosition.minus(robotTravel);
    }

}
