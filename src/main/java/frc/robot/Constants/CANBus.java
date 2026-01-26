package frc.robot.Constants;


public class CANBus {
   
    public static final class IntakeIDS {
        public final static int INTAKE = 0; //Place Holder
        public final static int PIVOT = 0; //Place Holder
    }

    public static final class ShooterIDs {
        /** the left one will be the leader */
        public final static int LEFT_MOTOR_ID = 0;
        /** the right one will be the follower */
        public final static int RIGHT_MOTOR_ID = 0;
    }

    public static final class DriveIDs {
        public final static int FRONT_LEFT_DRIVE = 7;
        public final static int FRONT_LEFT_TURN = 8;
        public final static int FRONT_LEFT_ENCODER = 4;

        public final static int FRONT_RIGHT_DRIVE = 1;
        public final static int FRONT_RIGHT_TURN = 2;
        public final static int FRONT_RIGHT_ENCODER = 3;

        public final static int BACK_LEFT_DRIVE = 5;
        public final static int BACK_LEFT_TURN = 4;
        public final static int BACK_LEFT_ENCODER = 1;

        public final static int BACK_RIGHT_DRIVE = 3;
        public final static int BACK_RIGHT_TURN = 6;
        public final static int BACK_RIGHT_ENCODER = 2;
    }
}
