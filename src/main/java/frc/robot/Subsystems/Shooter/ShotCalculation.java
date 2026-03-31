package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Measured.ShooterMeasurements;
import frc.robot.Constants.Tunables.FieldTunables;
import frc.robot.Constants.Tunables.ShooterTunables;
import java.util.Arrays;

public final class ShotCalculation {
    public static Translation2d getNearestHubPosition(Translation2d robotPosition) {
        return robotPosition.nearest(
                Arrays.asList(FieldMeasurements.BLUE_HUB_CENTER, FieldMeasurements.RED_HUB_CENTER));
    }

    public static Translation2d getNearestPassTargetPosition(Translation2d robotPosition) {
        return robotPosition.nearest(Arrays.asList(
                FieldTunables.BLUE_BOTTOM_PASSING_TARGET,
                FieldTunables.BLUE_TOP_PASSING_TARGET,
                FieldTunables.RED_BOTTOM_PASSING_TARGET,
                FieldTunables.RED_TOP_PASSING_TARGET));
    }

    /**
     * predicts where the hub will be after the robot moves during a shot
     */
    public static Translation2d calculateNextTargetPosition(
            Translation2d robotPosition, Translation2d targetPosition, ChassisSpeeds robotVelocity) {

        // use the distance from the hub to calculate the flight time
        double distanceFromTarget = robotPosition.getDistance(targetPosition);
        double flightTime = ShooterMeasurements.distanceToFlightTime(distanceFromTarget);

        // calculate how far the robot moves during that time
        Translation2d robotTravel = new Translation2d(
                robotVelocity.vxMetersPerSecond * flightTime, robotVelocity.vyMetersPerSecond * flightTime);

        // move the hub in the opposite direction of the robot travel to simulate were it will be after the flight time
        return targetPosition.minus(robotTravel);
    }

    /**
     * since the flight time changes when the hub position changes, we recalculate the values several times to converge
     * on an ideal target
     */
    public static Translation2d calculateTargetPositionMultiStep(
            Translation2d robotPosition, ChassisSpeeds robotVelocity, Translation2d targetPosition, int steps) {

        // the first position is the actual position
        Translation2d adjustedPosition = targetPosition;

        for (int stepsDone = 0; stepsDone < steps; stepsDone++) {
            // calculate the next position based on the previous position
            adjustedPosition = calculateNextTargetPosition(robotPosition, adjustedPosition, robotVelocity);
        }

        return adjustedPosition;
    }

    /** returns the hub target that robot should shoot at to compensate for its velocity */
    public static Translation2d getHubTarget(Translation2d robotPosition, ChassisSpeeds robotVelocity) {
        return calculateTargetPositionMultiStep(
                robotPosition,
                robotVelocity,
                getNearestHubPosition(robotPosition),
                ShooterTunables.SHOT_PREDICTION_ITERATIONS);
    }

    /** returns the pass target that robot should shoot at to compensate for its velocity */
    public static Translation2d getPassTarget(Translation2d robotPosition, ChassisSpeeds robotVelocity) {
        return calculateTargetPositionMultiStep(
                robotPosition,
                robotVelocity,
                getNearestPassTargetPosition(robotPosition),
                ShooterTunables.SHOT_PREDICTION_ITERATIONS);
    }

    /** uses the distance between the two translations to calculate the needed RPM of the shooter */
    public static double calculateRPM(Translation2d robotPosition, Translation2d targetPosition) {
        return ShooterMeasurements.distanceToFlywheelRPM(robotPosition.getDistance(targetPosition));
    }

    /** returns the RPM needed to shoot into the nearest Hub */
    public static double calculateNearestHubRPM(Translation2d robotPosition){
        return calculateRPM(robotPosition, getNearestHubPosition(robotPosition));
    }

    public static boolean calculateShotConfidence(
            Translation2d robotPosition, Translation2d originalTargetPosition, Translation2d calculatedTargetPosition) {
        double targetDistance = robotPosition.getDistance(originalTargetPosition);
        double calculatedTargetDistance = robotPosition.getDistance(calculatedTargetPosition);

        double distanceChange = Math.abs(targetDistance - calculatedTargetDistance);

        return distanceChange <= ShooterTunables.SHOOTING_POSITION_TOLERANCE;
    }
}
