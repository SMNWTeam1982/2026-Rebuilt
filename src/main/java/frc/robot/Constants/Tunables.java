package frc.robot.Constants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Tunables {
    public static final class DriveBaseTunables{
        /** the speed we limit the drive to, this MUST be below the physical max speed */
        public static final double ARTIFICIAL_MAX_SPEED = 3.0;

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
}