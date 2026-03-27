package frc.robot.Constants;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.Measured.FieldMeasurements;

// the rev website recommends a limit of 40A-60A for NEO 1.1
public class Tunables {

    /** a spark max config for motors that aren't doing a lot of work, set to brake on idle */
    public static final SparkBaseConfig DEFAULT_SPARK_MAX_CONFIG =
            new SparkMaxConfig().smartCurrentLimit(20).secondaryCurrentLimit(30).idleMode(IdleMode.kBrake);

    public static final String SPECIAL_MESSAGE = "quals";

    /** speeds are in meters per second */
    public static final class DriveBaseTunables {
        /** the speed we limit the drive to, this MUST be below the physical max speed */
        public static final double ARTIFICIAL_MAX_SPEED = 3.0;

        /**
         * configurable based on driver preference and game need
         * <p> this will be capped by the artificial max speed, so it can be set to any value & used to tune sensitivity
         * <p> meters/sec */
        public static final double DRIVE_SPEED = 2.5; // 1 m/s
        /** this can end up being capped by the artificial max speed, but the cap depends on robot size & module positions
         * <p> radians/sec
         */
        public static final double TURN_SPEED = -4.0; // 3 rad/s
        /** the amount the joystick needs to deflect before it will register an input */
        public static final double INPUT_DEADZONE = 0.1;

        /** a speed for the commands that fine tune the robot position robot-relative */
        public static final double NUDGE_SPEED = 0.5;

        public static final double MAX_AUTO_SPEED = 1.5;

        public static final double HEADING_P = 6.0;
        public static final double HEADING_I = 0.0;
        public static final double HEADING_D = 0.2;

        public static final Rotation2d HEADING_TOLERANCE = Rotation2d.fromDegrees(10);

        public static final double TRANSLATION_P = 4.0;
        public static final double TRANSLATION_I = 0.0;
        public static final double TRANSLATION_D = 0.0;

        public static final double TRANSLATION_TOLERANCE = 0.02; // 2cm
    }

    public static final class SwerveModuleTunables {
        // pid gains for the turn motor
        public static final double TURN_P = 0.85;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.02;

        /** config for the drive motor on the module */
        public static final SparkBaseConfig DRIVE_MOTOR_CONFIG =
                new SparkMaxConfig().smartCurrentLimit(40).inverted(true).idleMode(SparkBaseConfig.IdleMode.kBrake);

        /** config for the turn motor on the module */
        public static final SparkBaseConfig TURN_MOTOR_CONFIG =
                new SparkMaxConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kCoast);
    }

    public static final class FieldTunables {
        /** the distance from the hub that we have to be in order to score */
        public static final double HUB_SCORING_DISTANCE = 2.5;

        private static Translation2d flipX(Translation2d position) {
            // subtract the position from the other side of the field to get the flipped position
            double new_x = FieldMeasurements.FIELD_CENTER.getX() * 2 - position.getX();
            return new Translation2d(new_x, position.getY());
        }

        private static Translation2d flipY(Translation2d position) {
            // subtract the position from the other side of the field to get the flipped position
            double new_y = FieldMeasurements.FIELD_CENTER.getY() * 2 - position.getY();
            return new Translation2d(position.getX(), new_y);
        }

        public static final Translation2d BLUE_BOTTOM_PASSING_TARGET = new Translation2d(3, 2);

        public static final Translation2d RED_BOTTOM_PASSING_TARGET = flipX(BLUE_BOTTOM_PASSING_TARGET);

        public static final Translation2d BLUE_TOP_PASSING_TARGET = flipY(BLUE_BOTTOM_PASSING_TARGET);

        public static final Translation2d RED_TOP_PASSING_TARGET = flipY(RED_BOTTOM_PASSING_TARGET);
    }

    public static final class ShooterTunables {
        public static final double FLYWHEEL_P = 0.001; // 0.0008;
        public static final double FLYWHEEL_I = 0.0;
        public static final double FLYWHEEL_D = 0.00005; // 0.00007;

        public static final double FLYWHEEL_S = 0;
        public static final double FLYWHEEL_V = 0.128; // 26 Feb 2026
        public static final double FLYWHEEL_A = 0;

        public static final double FLYWHEEL_RPM_TOLERANCE = 50.0;

        public static final double FLYWHEEL_IDLE_RPM = 1200.0;

        public static final double SHOOTER_RPM_CEILING = 5500;

        public static final AngularVelocity SPEED_OVERRIDE_1 = RPM.of(0);
        public static final AngularVelocity SPEED_OVERRIDE_2 = RPM.of(3400);
        public static final AngularVelocity SPEED_OVERRIDE_3 = RPM.of(3600);
        public static final AngularVelocity SPEED_OVERRIDE_4 = RPM.of(3800);

        /** the maximum deviation from the ideal shooting position where the shot can still be made */
        public static final double SHOOTING_POSITION_TOLERANCE = 0.1;

        // public static final Rotation2d SHOOTING_ANGLE_TOLERANCE = Rotation2d.fromDegrees(15);

        public static final int SHOT_PREDICTION_ITERATIONS = 2;

        public static final Time AUTO_SPIN_UP_TIME = Seconds.of(3);

        // brake mode on for the flywheels so that we dont get a penalty for an eranious shot while the flywheels spin down
        // the brake mode should also help prevent jams
        // the total flywheel current should not exceed 60A (30A * 2 motors)
        // being somewhat conservative with the flywheel current limits
        public static final SparkBaseConfig FLYWHEEL_MOTOR_CONFIG = new SparkMaxConfig()
                .smartCurrentLimit(30)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .secondaryCurrentLimit(40);
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

        /** if the absolute position of the intake encoder is LESS than this the intake is considered STOWED */
        public static final double STOWED_THRESHOLD = 0.0;

        /** if the absolute position of the intake encoder is GREATER than this the intake is considered DEPLOYED */
        public static final double DEPLOYED_THRESHOLD = 0.0;

        /** how long does the intake position need to be past the stowed/deployed threshold before it considers it activated
         * <p> this will be used to debounce the stowed/deployed Trigger
         */
        public static final Time THRESHOLD_TIME = Seconds.of(0.2);

        /** the maximuma mount of time that the intake will run the pivot motor during a deploy attempt */
        public static final Time DEPLOY_ATTEMPT_TIME = Seconds.of(1.2);

        /** the maximuma mount of time that the intake will run the pivot motor during a retract attempt */
        public static final Time RETRACT_ATTEMPT_TIME = Seconds.of(1.2);

        public static final double MOVE_IN_SPEED = -0.2;
        public static final double MOVE_OUT_SPEED = 0.2;

        // percent that the intake will be set at when intaking
        public static final double INTAKE_SPEED = 0.85;

        public static final SparkBaseConfig PIVOT_MOTOR_CONFIG = new SparkMaxConfig()
                .smartCurrentLimit(40)
                .secondaryCurrentLimit(60)
                .idleMode(SparkBaseConfig.IdleMode.kCoast);
    }

    public static final class KickerTunables {
        /** the speed of the kicker when on high */
        public static final double HIGH_SPEED = 0.2;
        /** the speed of the kicker when on low */
        public static final double LOW_SPEED = -0.9;
        /** how long the kicker runs at the high speed before switching to the low speed */
        public static final Time HIGH_TIME = Seconds.of(0.5);
        /** how long the kicker runs at the low speed before switching to the high speed */
        public static final Time LOW_TIME = Seconds.of(0.2);
        /** the speed the kicker runs at when not active */
        public static final double IDLE_SPEED = 0.0;

        public static final SparkBaseConfig KICKER_MOTOR_CONFIG = new SparkMaxConfig()
                .smartCurrentLimit(20)
                .secondaryCurrentLimit(30)
                .idleMode(IdleMode.kCoast);
    }

    public static final class ClimberTunables {
        public static final double EXTEND_SPEED = 0.0;
        public static final double RETRACT_SPEED = 0.0;
    }

    public static final class VisionTunables {
        public static final Matrix<N3, N1> STANDARD_DEVIATIONS = VecBuilder.fill(.7, .7, .3);
        public static final Matrix<N3, N1> MULTI_TAG_STANDARD_DEVIATIONS = VecBuilder.fill(0.15, 0.15, 0.1);
    }
}
