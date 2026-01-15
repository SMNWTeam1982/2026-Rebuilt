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
}