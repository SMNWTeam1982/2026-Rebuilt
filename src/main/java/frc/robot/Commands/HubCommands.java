package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Measured.FieldMeasurements;

public final class HubCommands {
    public static Translation2d calculateScoringTarget(
            Translation2d robotPosition, Translation2d hubPosition, ChassisSpeeds robotVelocity) {
        double distanceFromHub = robotPosition.getDistance(hubPosition);

        double flightTime = FieldMeasurements.hubDistanceToFlightTime(distanceFromHub);

        Translation2d robotTravel = new Translation2d(
                robotVelocity.vxMetersPerSecond * flightTime, robotVelocity.vyMetersPerSecond * flightTime);

        // move the hub in the opposite direction of the robots travel to simulate where it going to be when the ball
        // lands
        return hubPosition.minus(robotTravel);
    }
}
