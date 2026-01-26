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

    public static Translation2d calculateScoringTargetMultiStep(
            Translation2d robotPosition, Translation2d hubPosition, ChassisSpeeds robotVelocity, int steps) {

        // the first position is the actual position of the hub
        if (steps == 0) {
            return hubPosition;
        }

        // get the previous position
        Translation2d previousHubPosition =
                calculateScoringTargetMultiStep(robotPosition, hubPosition, robotVelocity, steps - 1);

        // use the previous position to calculate the new flight time
        double distanceFromHub = robotPosition.getDistance(previousHubPosition);
        double flightTime = FieldMeasurements.hubDistanceToFlightTime(distanceFromHub);

        // calculate new robot travel
        Translation2d robotTravel = new Translation2d(
                robotVelocity.vxMetersPerSecond * flightTime, robotVelocity.vyMetersPerSecond * flightTime);

        // return the new hub position
        return hubPosition.minus(robotTravel);
    }
}
