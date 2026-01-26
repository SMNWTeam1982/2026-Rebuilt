package frc.robot.Constants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class Tunables {
    /** speeds are in meters per second */
    public static final class DriveBaseTunables{
        /** the speed we limit the drive to, this MUST be below the physical max speed */
        public static final double ARTIFICIAL_MAX_SPEED = 3.0;
        /** a speed for the commands that fine tune the robot position robot-relative */
        public static final double NUDGE_SPEED = 0.25;

        public static final double MAX_AUTO_SPEED = 0.5;
        public static final double AUTO_TRANSLATION_TOLERANCE = 0.02; // 2cm
        public static final Rotation2d AUTO_ROTATION_TOLERANCE = Rotation2d.fromDegrees(1);

        public static final double HEADING_P = 3.5;
        public static final double HEADING_I = 0.0;
        public static final double HEADING_D = 0.0;

        public static final double TRANSLATION_P = 3.0;
        public static final double TRANSLATION_I = 0.0;
        public static final double TRANSLATION_D = 0.0;
    }

    public static final class SwerveModuleTunables{
        // pid gains for the turn motor
        public static final double TURN_P = 0.73;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.01;

        /** config for the drive motor on the module */
        public static final SparkBaseConfig DRIVE_MOTOR_CONFIG = new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kBrake);
        /** config for the turn motor on the module */
        public static final SparkBaseConfig TURN_MOTOR_CONFIG = new SparkMaxConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast);
    }

    public static final class FieldTunables{
        /** the distance from the hub that we have to be in order to score */
        public static final double HUB_SCORING_DISTANCE = 2.5;
    }

    public static final class ShooterTunables{
        public static final double FLYWHEEL_P = 0.0;
        public static final double FLYWHEEL_I = 0.0;
        public static final double FLYWHEEL_D = 0.0;

        public static final double FLYWHEEL_RPM_TOLERANCE = 50.0;

        public static final double FLYWHEEL_IDLE_RPM = 500.0;

        // the flywheels should coast when disables so the motors don't have to absorb all of the momentum
        // the total flywheel current should not exceed 50A (25A * 2 motors)
        public static final SparkBaseConfig FLYWHEEL_MOTOR_CONFIG = new SparkMaxConfig().smartCurrentLimit(25).idleMode(SparkBaseConfig.IdleMode.kCoast);
    }
}