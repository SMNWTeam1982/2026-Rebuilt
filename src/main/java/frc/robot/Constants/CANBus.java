package frc.robot.Constants;

public class CANBus {

    public static final class DriveIDs {
        public static final int FRONT_LEFT_DRIVE = 1;
        public static final int FRONT_LEFT_TURN = 2;
        public static final int FRONT_LEFT_ENCODER = 4;

        public static final int FRONT_RIGHT_DRIVE = 3;
        public static final int FRONT_RIGHT_TURN = 4;
        public static final int FRONT_RIGHT_ENCODER = 3;

        public static final int BACK_LEFT_DRIVE = 5;
        public static final int BACK_LEFT_TURN = 6;
        public static final int BACK_LEFT_ENCODER = 1;

        public static final int BACK_RIGHT_DRIVE = 7;
        public static final int BACK_RIGHT_TURN = 8;
        public static final int BACK_RIGHT_ENCODER = 2;
    }

    public static final class IntakeIDs {
        public static final int INTAKE = 9;
        public static final int PIVOT = 10;

        public static final int PIVOT_ENCODER = 0;
    }

    public static final class ShooterIDs {
        /** the left one will be the leader */
        public static final int LEFT_MOTOR_ID = 11;
        /** the right one will be the follower */
        public static final int RIGHT_MOTOR_ID = 12;
    }

    public static final class ClimberIDs {
        public static final int CLIMBER = 13;
    }

    public static final class KickerIDs {
        public static final int KICKER = 14;
    }

    public static final class VisionConstants {
        public static final String limeLightCameraName = "limelight-front";
    }
}
