package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Measured.ShooterMeasurements;
import frc.robot.Constants.Tunables.ShooterTunables;
import java.util.Arrays;

public final class ShotCalculation {
    public static Translation2d getNearestHubPosition(Translation2d robotPosition) {
        return robotPosition.nearest(
                Arrays.asList(FieldMeasurements.BLUE_HUB_CENTER, FieldMeasurements.RED_HUB_CENTER));
    }

    /**
     * predicts where the hub will be after the robot moves during a shot
     */
    public static Translation2d calculateNextHubPosition(
            Translation2d robotPosition, Translation2d hubPosition, ChassisSpeeds robotVelocity) {

        // use the distance from the hub to calculate the flight time
        double distanceFromHub = robotPosition.getDistance(hubPosition);
        double flightTime = ShooterMeasurements.hubDistanceToFlightTime(distanceFromHub);

        // calculate how far the robot moves during that time
        Translation2d robotTravel = new Translation2d(
                robotVelocity.vxMetersPerSecond * flightTime, robotVelocity.vyMetersPerSecond * flightTime);

        // move the hub in the opposite direction of the robot travel to simulate were it will be after the flight time
        return hubPosition.minus(robotTravel);
    }

    /**
     * since the flight time changes when the hub position changes, we recalculate the values several times to converge
     * on an ideal target
     */
    public static Translation2d calculateHubPositionMultiStep(
            Translation2d robotPosition, ChassisSpeeds robotVelocity, int steps) {

        // the first position is the actual hub position
        Translation2d hubPosition = getNearestHubPosition(robotPosition);

        for (int stepsDone = 0; stepsDone < steps; stepsDone++) {
            // calculate the next hub position based on the previous hub position
            hubPosition = calculateNextHubPosition(robotPosition, hubPosition, robotVelocity);
        }

        return hubPosition;
    }
}
