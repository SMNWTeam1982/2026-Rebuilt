package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;

public class Measured {
    public static final class DriveBaseMeasurements {
        public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.2635, 0.2635);
        public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.2635, -0.2635);
        public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-0.2635, 0.2635);
        public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-0.2635, -0.2635);

        /** the fastest speed the robot can go, derived from testing */
        public static final double PHYSICAL_MAX_SPEED = 3.8;

        /** used for discretization */
        public static final double DRIVE_PERIOD = TimedRobot.kDefaultPeriod;
    }

    public static final class SwerveModuleMeasurements {
        /** a number that is measured every year */
        public static final double POSITION_TO_METERS_MULTIPLIER = -0.31927 / 6.75;

        /** position to meters / 60seconds */
        public static final double RPM_TO_MPS_MULTIPLIER = POSITION_TO_METERS_MULTIPLIER / 60;

        /** the minimum value that the drive motor has to be set to before it can move */
        public static final double DRIVE_STATIC_GAIN = 0.05;

        /** multiplier that converts a velocity to a voltage to feed to the drive motor */
        public static final double DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER = 2.87;
    }

    public static final class FieldMeasurements {
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d();
        public static final Translation2d RED_HUB_CENTER = new Translation2d();

        public static final double MAX_HUB_SCORING_DISTANCE = 4.5;
        public static final double MIN_HUB_SCORING_DISTANCE = 1.5;
    }

    public static final class ShooterMeasurements {

        /**
         * the equation will be derived from a best fit of a data table that will be measured, expected
         * to be quadratic or cubic
         */
        public static double hubDistanceToFlywheelRPM(double distanceFromHub) {
            return 0.0;
        }

        /**
         * the equation will be derived from a best fit of a data table that will be measured, expected
         * to be linear
         */
        public static double hubDistanceToFlightTime(double distanceFromHub) {
            return 0.0;
        }
    }
}
