package frc.robot.Constants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Tunables {
    /** speeds are in meters per second */
    public static final class DriveBaseTunables {
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

    public static final class SwerveModuleTunables {
        // pid gains for the turn motor
        public static final double TURN_P = 0.73;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.01;

        /** config for the drive motor on the module */
        public static final SparkBaseConfig DRIVE_MOTOR_CONFIG =
                new SparkMaxConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kBrake);

        /** config for the turn motor on the module */
        public static final SparkBaseConfig TURN_MOTOR_CONFIG =
                new SparkMaxConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast);
    }

    public static final class FieldTunables {
        /** the distance from the hub that we have to be in order to score */
        public static final double HUB_SCORING_DISTANCE = 2.5;
    }

    public static final class ShooterTunables {
        public static final double FLYWHEEL_P = 0.0016;
        public static final double FLYWHEEL_I = 0.0;
        public static final double FLYWHEEL_D = 0.0002;

        public static final double FLYWHEEL_S = 0;
        public static final double FLYWHEEL_V = 0.13;
        public static final double FLYWHEEL_A = 0;

        public static final double FLYWHEEL_RPM_TOLERANCE = 25.0;

        public static final double FLYWHEEL_IDLE_RPM = 2000.0;

        public static final double SHOOTER_RPM_CEILING = 5600;

        /** the maximum deviation from the ideal shooting position where the shot can still be made */
        public static final double SHOOTING_POSITION_TOLERANCE = 0.1;

        public static final int SHOT_PREDICTION_ITERATIONS = 2;

        // the flywheels should coast when disables so the motors don't have to absorb all of the momentum
        // the rev website recommends a limit of 40A-60A for NEO 1.1
        // the total flywheel current should not exceed 80A (40A * 2 motors)
        public static final SparkBaseConfig FLYWHEEL_MOTOR_CONFIG = new SparkMaxConfig()
                .smartCurrentLimit(40)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .secondaryCurrentLimit(60);
    }

    public static final class IntakeTunables {
        // pid gains for the deploy/retraction of the intake
        public static final double PIVOT_P = 1.0;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.0;

        public static final Rotation2d PIVOT_TOLERANCE = Rotation2d.fromDegrees(1);

        // feedforward gains for the gravity compensation
        public static final double PIVOT_S = 0.0;
        public static final double PIVOT_G = 0.0;
        public static final double PIVOT_V = 0.0;

        public static final Rotation2d STOW_POSITION = new Rotation2d();
        public static final Rotation2d DEPLOY_POSITION = new Rotation2d();

        // percent that the intake will be set at when intaking
        public static final double INTAKE_SPEED = 0.0;

        public static final SparkBaseConfig PIVOT_MOTOR_CONFIG =
                new SparkMaxConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kBrake);
    }

    public static final class KickerTunables {
        public static final double KICKER_SPEED = 0.0;
    }

    public static final class ClimberTunables {
        public static final double EXTEND_SPEED = 0.0;
        public static final double RETRACT_SPEED = 0.0;
    }

    public static final class VisionTunables {
        public static final Transform3d PHOTON_CAM_RELATIVE_TO_ROBOT = new Transform3d(
                new Translation3d(Units.inchesToMeters(12.0), Units.inchesToMeters(0.0), Units.inchesToMeters(9.75)),
                new Rotation3d(0.0, 10.0, 0.0));

        public static final Matrix<N3, N1> PHOTON_CAM_VISION_TRUST = VecBuilder.fill(0.5, 0.5, 1);
    }
}
