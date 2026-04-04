package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.Tunables.ShooterTunables;

public class Measured {
    public static final class DriveBaseMeasurements {
        // 0.271653
        public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.271653, 0.271653);
        public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.271653, -0.271653);
        public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-0.271653, 0.271653);
        public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-0.271653, -0.271653);

        /** the fastest speed the robot can go, derived from testing */
        public static final double PHYSICAL_MAX_SPEED = 3.8;

        /** used for discretization */
        public static final double DRIVE_PERIOD = TimedRobot.kDefaultPeriod;
    }

    public static final class PathplannerMeasurements {
        private static final Distance WHEEL_RADIUS = Inches.of(2); // 12.5/2pi is about 2 inches
        private static final LinearVelocity MAX_DRIVE_VELOCITY =
                MetersPerSecond.of(DriveBaseMeasurements.PHYSICAL_MAX_SPEED); // set elsewhere
        private static final double WHEEL_COF = 1.2; // pathplanner default
        private static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1); // 1 drive neo
        private static final Current DRIVE_CURRENT_LIMIT = Amps.of(40); // set elsewhere
        private static final ModuleConfig MODULE_CONFIG =
                new ModuleConfig(WHEEL_RADIUS, MAX_DRIVE_VELOCITY, WHEEL_COF, DRIVE_MOTOR, DRIVE_CURRENT_LIMIT, 1);

        private static final Mass ROBOT_MASS = Kilograms.of(52); // max robot weight
        private static final MomentOfInertia ROBOT_MOMENT_OF_INERTIA =
                KilogramSquareMeters.of(4.08); // uses pathplanner's moi estimate equation

        public static final RobotConfig PATHPLANNER_CONFIG = new RobotConfig(
                ROBOT_MASS,
                ROBOT_MOMENT_OF_INERTIA,
                MODULE_CONFIG,
                DriveBaseMeasurements.FRONT_LEFT_TRANSLATION,
                DriveBaseMeasurements.FRONT_RIGHT_TRANSLATION,
                DriveBaseMeasurements.BACK_LEFT_TRANSLATION,
                DriveBaseMeasurements.BACK_RIGHT_TRANSLATION);
    }

    public static final class SwerveModuleMeasurements {
        /** will change based on the type of tread used on the swerve module,
         * <p> swerve wheel circumference divided by the drive gear ratio
         * <p> distance traveled by the wheel per rotation of the drive motor
         */
        public static final double DRIVE_ENCODER_POSITION_TO_METERS_MULTIPLIER =
                Units.inchesToMeters(12.8) / 6.75; // measured on march 7 2026, old 2025 number: -0.31927 / 6.75;
        // swerve treads changed for GKC, old circumference = 12.5 inches

        /** position to meters / 60seconds */
        public static final double DRIVE_ENCODER_RPM_TO_MPS_MULTIPLIER =
                DRIVE_ENCODER_POSITION_TO_METERS_MULTIPLIER / 60;

        /** max NEO 1.1 rpm * rpm->mps conversion factor, 2026 value: 4.45 mps */
        public static final double THEORETICAL_MAX_VELOCITY =
                5676 * DRIVE_ENCODER_RPM_TO_MPS_MULTIPLIER; // approx 4.45mps, exact: 4.4497037037

        /** the minimum value that the drive motor has to be set to before it can move */
        public static final double DRIVE_STATIC_GAIN = 0.05;

        /** multiplier that converts a velocity to a voltage to feed to the drive motor */
        public static final double DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER = 2.87;
    }

    public static final class FieldMeasurements {
        public static final Translation2d FIELD_CENTER =
                new Translation2d(Units.inchesToMeters(325.06), Units.inchesToMeters(158.32));

        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(
                Units.inchesToMeters(181.56), // april tag 18 X
                Units.inchesToMeters(158.32) // april tag 26 Y
                );

        public static final Translation2d RED_HUB_CENTER = new Translation2d(
                Units.inchesToMeters(468.56), // april tag 5 X
                Units.inchesToMeters(158.32) // april tag 4 Y
                );

        public static final double MAX_HUB_SCORING_DISTANCE = 6; // approx March 7
        public static final double MIN_HUB_SCORING_DISTANCE = 2; // approx March 7
    }

    public static final class ShooterMeasurements {

        public static final double MAX_SCHOOL_FLYWHEEL_RPM =
                4750; // the balls will hit the cieling in the cafeteria above this speed
        public static final double HIGEST_RECORDED_FLYWHEEL_RPM_DROP = 400; // we recorded this at 4750 on march 7

        /// data table:
        ///
        /// distance | RPM | flight time
        ///
        /// AKit2: 26-03-24_23-27-29
        /// 2.005 | 2800 | no data
        /// 2.533 | 3000 | no data
        /// 3.606 | 3500 | no data
        /// ----------------------
        /// AKit1: 26-03-24_23-39-51
        /// 3.533 | 3400 | no data
        /// 3.982 | 3600 | no data
        /// 4.495 | 3800 | no data
        /// ----------------------
        ///
        /// linear regression for RPM (y) and distance (x)
        /// y = 407.53685x + 1981.08371
        ///
        /**
         * the equation will be derived from a best fit of a data table that will be measured, expected
         * to be quadratic or cubic
         */
        public static double distanceToFlywheelRPM(double distanceFromHub) {
            double x = distanceFromHub;
            double calculatedRPM = (407.53685 * x) + 1981.08371;
            return MathUtil.clamp(calculatedRPM, 0, ShooterTunables.SHOOTER_RPM_CEILING);
        }

        /// distance | RPM (based on the equation from above) | flight time
        /// St. Louis practice match 18
        /// 2.429 | 2970.886 | 1.10
        /// 2.861 | 3146.882 | 1.18
        /// 2.769 | 3109.546 | 1.15
        /// 2.775 | 3111.992 | 1.17
        /// 2.591 | 3037.112 | 1.15
        /// -----------------------
        /// St. Louis practic match 24
        /// 4.836 | 3951.    | 1.45 <- row not used
        /// 4.828 | 3948.865 | 1.47
        /// 3.567 | 3273.297 | 1.36
        /// 2.340 | 2934.641 | 1.02
        /// -----------------------
        ///
        /// linear regression for flight time (t) and distance (x)
        /// t = 0.180436x + 0.637674
        /**
         * the equation will be derived from a best fit of a data table that will be measured, expected
         * to be linear
         */
        public static double distanceToFlightTime(double distanceFromHub) {
            double x = distanceFromHub;
            double calculatedFlightTime = (0.180436 * x) + 0.637674;
            return Math.max(calculatedFlightTime, 0.0);
        }
    }

    public static final class IntakeMeasurements {
        public static final Rotation2d FULLY_RETRACTED_ANGLE = new Rotation2d();
        public static final Rotation2d FULLY_DEPLOYED_ANGLE = new Rotation2d();

        /** the absolute position of the intake encoder when fully retracted */
        public static final double RAW_FULLY_RETRACTED_POSITION = 0.0;
        /** the absolute position of the intake encoder when fully deployed */
        public static final double RAW_FULLY_DEPLOYED_POSITION = 0.0;
    }

    public static final class VisionMeasurements {
        public static final Transform3d PHOTON_CAM_RELATIVE_TO_ROBOT = new Transform3d(
                new Translation3d(Units.inchesToMeters(-2), Units.inchesToMeters(9.25), Units.inchesToMeters(20.375)),
                new Rotation3d(0.0, Math.toRadians(-11.0 /*9.0*/), 0.0));
    }
}
