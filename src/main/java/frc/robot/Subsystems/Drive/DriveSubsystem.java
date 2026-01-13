package frc.robot.Subsystems.Drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.VisionData;

public class DriveSubsystem extends SubsystemBase {

    public static final class DriveConstants {
        public static final double PHYSICAL_MAX_MPS = 3.8;

        public static final double ARTIFICIAL_MAX_MPS = 3.0;

        public static final double DRIVE_PERIOD = TimedRobot.kDefaultPeriod;
        public static final boolean GYRO_REVERSED = false;

        public static final double NUDGE_SPEED = .25;

        public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(3.5, 0.0, 0.0);

        public static final Constraints TRANSLATION_PID_CONSTRAINTS = new Constraints(1.0, 1.0);

        public static final double AUTO_MAX_SPEED = 0.5; // mps

        public static final double AUTO_TRANSLATION_TOLERANCE = 0.02; // 1cm

        public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(3.0, 0.0, 0.0);

        public static final Rotation2d AUTO_ROTATION_TOLERANCE = Rotation2d.fromDegrees(1);

        public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(0.2635, 0.2635);
        public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(0.2635, -0.2635);
        public static final Translation2d REAR_LEFT_TRANSLATION = new Translation2d(-0.2635, 0.2635);
        public static final Translation2d REAR_RIGHT_TRANSLATION = new Translation2d(-0.2635, -0.2635);

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                FRONT_LEFT_TRANSLATION, FRONT_RIGHT_TRANSLATION, REAR_LEFT_TRANSLATION, REAR_RIGHT_TRANSLATION);
    }

    public static class PathPlannerConstants {
        public static final ModuleConfig MODULE_CONFIG =
                new ModuleConfig(0.048, DriveConstants.ARTIFICIAL_MAX_MPS, 1.200, DCMotor.getNEO(1), 30, 1);
        public static final Translation2d[] MODULE_OFFSETS = new Translation2d[] {
            DriveConstants.FRONT_LEFT_TRANSLATION,
            DriveConstants.FRONT_RIGHT_TRANSLATION,
            DriveConstants.REAR_LEFT_TRANSLATION,
            DriveConstants.REAR_RIGHT_TRANSLATION
        };
        public static final Mass ROBOT_MASS = Kilogram.of(44.49741);
        public static final MomentOfInertia ROBOT_MOMENT_OF_INERTIA = KilogramSquareMeters.of(36.038);
        public static final RobotConfig PATHPLANNER_CONFIG =
                new RobotConfig(ROBOT_MASS, ROBOT_MOMENT_OF_INERTIA, MODULE_CONFIG, MODULE_OFFSETS);
    }

    private final SwerveModule frontLeft = new SwerveModule(7, 8, 4);
    private final SwerveModule frontRight = new SwerveModule(1, 2, 3);
    private final SwerveModule backLeft = new SwerveModule(5, 4, 1);
    private final SwerveModule backRight = new SwerveModule(3, 6, 2);

    /**
     * the rotation the gyro represents is not the same as the robot heading the gyro data is used as
     * a reference for the pose estimator the pose estimator offsets the gyro data to get the actual
     * robot heading
     *
     * <p>you do NOT have to zero the gyro
     */
    private final Pigeon2 gyro = new Pigeon2(0);

    private Field2d teleopField = new Field2d();
    private Field2d autoField = new Field2d();

    private final Supplier<VisionData> visionDataGetter;
    private final BooleanSupplier visionFreshnessGetter;

    /** the swerve drive is initialized with a default Pose2d, (0 x, 0 y, 0 rotation) */
    private final SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.SWERVE_KINEMATICS, gyro.getRotation2d(), getModulePositions(), new Pose2d());

    /**
     * this controller is used for "top down" robot control. give this controller the current heading
     * and the desired heading and it will output in radians per second
     */
    private final PIDController headingController = new PIDController(
            DriveConstants.ROTATION_PID_CONSTANTS.kP,
            DriveConstants.ROTATION_PID_CONSTANTS.kI,
            DriveConstants.ROTATION_PID_CONSTANTS.kD);

    private final PIDController xController = new PIDController(
            DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kI,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kD); // DriveConstants.TRANSLATION_PID_CONSTRAINTS);

    private final PIDController yController = new PIDController(
            DriveConstants.TRANSLATION_PID_CONSTANTS.kP,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kI,
            DriveConstants.TRANSLATION_PID_CONSTANTS.kD); // DriveConstants.TRANSLATION_PID_CONSTRAINTS);

    public DriveSubsystem(Supplier<VisionData> visionDataGetter, BooleanSupplier visionFreshnessGetter) {
        this.visionDataGetter = visionDataGetter;
        this.visionFreshnessGetter = visionFreshnessGetter;

        headingController.setTolerance(DriveConstants.AUTO_ROTATION_TOLERANCE.getRadians());
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(DriveConstants.AUTO_TRANSLATION_TOLERANCE);
        yController.setTolerance(DriveConstants.AUTO_TRANSLATION_TOLERANCE);

        configurePathPlanner();
        configureSwerveDriveLogging();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        swervePoseEstimator.update(gyro.getRotation2d(), getModulePositions());

        if (visionFreshnessGetter.getAsBoolean()) {
            VisionData lastVisionData = visionDataGetter.get();
            swervePoseEstimator.addVisionMeasurement(
                    lastVisionData.pose, lastVisionData.timestamp, lastVisionData.standardDeviations);
        }

        logRobotPose(getEstimatedPose());

        Logger.recordOutput("position x error", xController.getError());
        Logger.recordOutput("position y error", yController.getError());
        Logger.recordOutput("rotation error", headingController.getError());

        Logger.recordOutput("total drivetrain drive motor current draw", getTotalDriveMotorCurrentDraw());
        Logger.recordOutput("average drive motor current draw", getAverageDriveMotorCurrentDraw());
    }

    /** puts information about about the swerve drive onto the dashboard */
    private void configureSwerveDriveLogging() {
        SmartDashboard.putData("Teleoperated Field", teleopField);
        SmartDashboard.putData("Autonomous Field", autoField);
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                builder.addDoubleProperty(
                        "Front Left Angle", () -> frontLeft.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(
                        "Front Right Angle", () -> frontRight.getState().angle.getRadians(), null);
                builder.addDoubleProperty(
                        "Front Right Velocity", () -> frontRight.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(
                        "Back Left Angle", () -> backLeft.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty(
                        "Back Right Angle", () -> backRight.getState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> backRight.getState().speedMetersPerSecond, null);
                builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
            }
        });
    }

    /** Init method for configuring PathPlanner to improve readability in constructor */
    private void configurePathPlanner() {
        // Log pathplanner poses and trajectories to custom Field2d object for visualization
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            autoField.getObject("Target").setPose(pose);
            Logger.recordOutput("Drive/Auto/TargetPose", pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poseList) -> {
            Pose2d[] poses = poseList.toArray(new Pose2d[poseList.size()]);
            if (poses.length != 0) {
                autoField.getObject("trajectory").setPoses(poses);
                Logger.recordOutput("Drive/Auto/Trajectory", poses);
            }
        });
        // the AutoBuilder will generate commands for us that follow a given trajectory.
        // the Commands it generates require a lot of complicated inputs so we supply these to the
        // AutoBuilder and it will input those automatically
        AutoBuilder.configure(
                this::getEstimatedPose,
                (pose) -> {}, // we pass in a dummy function because we are getting absolute position from
                // vision data
                this::getRobotRelativeSpeeds,
                (speeds) -> setModulesFromRobotRelativeSpeeds(speeds),
                new PPHolonomicDriveController(
                        DriveConstants.TRANSLATION_PID_CONSTANTS, // Translation PID constants
                        DriveConstants.ROTATION_PID_CONSTANTS // Rotation PID constants
                        ),
                PathPlannerConstants.PATHPLANNER_CONFIG,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // a reference to this subsystem
                );
    }

    /**
     * converts joystick inputs to field relative inputs so the robot moves relative to the drivers station it is viewed from
     * @param joystickSpeeds a chassisSpeeds with x and y being feed from the joystick values, and the theta coming from any source (usually the other stick)
     * @return
     */
    public ChassisSpeeds joystickSpeedsToFieldRelativeSpeeds(ChassisSpeeds joystickSpeeds, boolean onBlueSide) {
        if (onBlueSide) {
            return new ChassisSpeeds(
                    -joystickSpeeds.vyMetersPerSecond,
                    -joystickSpeeds.vxMetersPerSecond,
                    joystickSpeeds.omegaRadiansPerSecond);
        } else {
            return new ChassisSpeeds(
                    joystickSpeeds.vyMetersPerSecond,
                    joystickSpeeds.vxMetersPerSecond,
                    joystickSpeeds.omegaRadiansPerSecond);
        }
    }

    /**
     * Resets the pose estimator to the specified pose.
     *
     * <p>this will be useful for the questnav because as of 15 sep 2025 questnav is relative
     *
     * <p>this will be irrelavant if we are using absolute position
     */
    public Command resetEstimatedPose(Pose2d pose) {
        return runOnce(() -> {
                    swervePoseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
                });
    }

    /**
     * Zeroes the heading of the pose estimator
     *
     * <p>anything that resets the pose estimator will be irrelavant with absolute position
     */
    public Command zeroEstimatedHeading() {
        return runOnce(() -> {
            swervePoseEstimator.resetRotation(new Rotation2d());
        });
    }

    /** sets all of the drivetrain motors to 0 */
    public Command stop() {
        return runOnce(() -> {
            frontLeft.stop();
            frontRight.stop();
            backLeft.stop();
            backRight.stop();
        });
    }

    /** drives the robot with chassis speeds relative to the robot coordinate system */
    public Command driveRobotRelative(Supplier<ChassisSpeeds> desiredRobotSpeeds) {
        return run(() -> {
            setModulesFromRobotRelativeSpeeds(desiredRobotSpeeds.get());
        });
    }

    /** drives the robot with chassis speeds relative to the field coordinate system */
    public Command driveFieldRelative(Supplier<ChassisSpeeds> desiredFieldSpeeds) {
        return run(() -> {
            ChassisSpeeds convertedSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(desiredFieldSpeeds.get(), getHeading());
            setModulesFromRobotRelativeSpeeds(convertedSpeeds);
        });
    }

    /**
     * relative to the drivers station -y is away an +x is to the right, this is so you can just
     * multiply the joystick output by the desired speed
     *
     * <p>see the wpilib coordinate system page, the controller's x and y are wierd
     *
     * <p>up on a joystick is -y and right on a joystick is +x
     *
     * @param onBlueSide controls if this factory returns a command for blue or a command for red, to
     *     change the controls this function will need to be called agian
     */
    public Command driveFromDriversStation(Supplier<ChassisSpeeds> driversStationRelativeSpeeds, boolean onBlueSide) {
        if (onBlueSide) {
            return driveFieldRelative(() -> {
                ChassisSpeeds driversSpeeds = driversStationRelativeSpeeds.get();
                ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
                fieldRelativeSpeeds.vxMetersPerSecond = -driversSpeeds.vyMetersPerSecond;
                fieldRelativeSpeeds.vyMetersPerSecond = -driversSpeeds.vxMetersPerSecond;
                fieldRelativeSpeeds.omegaRadiansPerSecond = driversSpeeds.omegaRadiansPerSecond;

                return fieldRelativeSpeeds;
            });
        } else {
            return driveFieldRelative(() -> {
                ChassisSpeeds driversSpeeds = driversStationRelativeSpeeds.get();
                ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
                fieldRelativeSpeeds.vxMetersPerSecond = driversSpeeds.vyMetersPerSecond;
                fieldRelativeSpeeds.vyMetersPerSecond = driversSpeeds.vxMetersPerSecond;
                fieldRelativeSpeeds.omegaRadiansPerSecond = driversSpeeds.omegaRadiansPerSecond;

                return fieldRelativeSpeeds;
            });
        }
    }

    /**
     * drives the robot like a top-down shooter
     *
     * @param desiredTranslation the theta component is ignored
     *
     * <p>the x & y velocities and the field rotation are relative to the field coordinate system
     */
    public Command driveTopDown(Supplier<ChassisSpeeds> desiredTraslation, Supplier<Rotation2d> desiredFieldRotation) {
        return runEnd(
                () -> {
                    double pidOutput = headingController.calculate(
                            getHeading().getRadians(),
                            desiredFieldRotation.get().getRadians());

                    ChassisSpeeds translations = desiredTraslation.get();

                    ChassisSpeeds finalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            translations.vxMetersPerSecond, translations.vyMetersPerSecond, pidOutput, getHeading());

                    setModulesFromRobotRelativeSpeeds(finalSpeeds);
                },
                () -> {
                    headingController.reset();
                });
    }

    /**
     * creates a command to move the robot to a specific pose
     * <p>note that the target pose doesnt change when the command is run multiple times
     */
    public Command moveToPose(Pose2d targetPose) {
        return runOnce(() -> autoField.getObject("Target").setPose(targetPose))
                .andThen(driveTopDown(
                                () -> {
                                    double xOutput = -xController.calculate(
                                            getEstimatedPose().getX(), targetPose.getX());
                                    double clampedXOutput = MathUtil.clamp(
                                            xOutput, -DriveConstants.AUTO_MAX_SPEED, DriveConstants.AUTO_MAX_SPEED);

                                    double yOutput = -yController.calculate(
                                            getEstimatedPose().getY(), targetPose.getY());
                                    double clampedYOutput = MathUtil.clamp(
                                            yOutput, -DriveConstants.AUTO_MAX_SPEED, DriveConstants.AUTO_MAX_SPEED);

                                    return new ChassisSpeeds(clampedXOutput, clampedYOutput, 0);
                                },
                                () -> targetPose.getRotation())
                        .until(() -> xController.atSetpoint() && yController.atSetpoint()))
                .finallyDo(() -> {
                    xController.reset();
                    yController.reset();
                });
    }
    /**
     * a command for debugging a specific module, note that because this command affects only one
     * module, you can only debug one module at a time
     *
     * @param moduleIndex 0 = fl, 1 = fr, 2 = bl, 3 = br
     * @param driveAmount -1.0 - +1.0 scale for setting the motor
     * @param turnAmount -1.0 - +1.0 scale for setting the motor
     */
    public Command runModule(int moduleIndex, DoubleSupplier driveAmount, DoubleSupplier turnAmount) {
        return run(() -> {
            if (moduleIndex == 0) {
                frontLeft.runMotors(driveAmount.getAsDouble(), turnAmount.getAsDouble());
            }
            if (moduleIndex == 1) {
                frontRight.runMotors(driveAmount.getAsDouble(), turnAmount.getAsDouble());
            }
            if (moduleIndex == 2) {
                backLeft.runMotors(driveAmount.getAsDouble(), turnAmount.getAsDouble());
            }
            if (moduleIndex == 3) {
                backRight.runMotors(driveAmount.getAsDouble(), turnAmount.getAsDouble());
            }
        });
    }

    /**
     * calculates the needed module states and normalizes the wheel velocities
     *
     * <p>this will automatically slow down the speed to one that is possible by the swerve drive
     * modules
     */
    private void setModulesFromRobotRelativeSpeeds(ChassisSpeeds speeds) {
        Logger.recordOutput("final robot relative speeds", speeds);
        ChassisSpeeds.discretize(speeds, DriveConstants.DRIVE_PERIOD);
        SwerveModuleState[] moduleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.ARTIFICIAL_MAX_MPS);
        setModuleStates(moduleStates);
    }

    /**
     * calls setDesiredState() on each of the swerve modules
     *
     * @param desiredStates [front left, front right, back left, back right]
     */
    private void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Returns the heading from getEstimatedPose() */
    public Rotation2d getHeading() {
        return getEstimatedPose().getRotation();
    }

    /**
     * gets the estimated position from the pose estimator.
     *
     * <p>this returns the same thing per call
     */
    public Pose2d getEstimatedPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    /** gets the last desired state that the module got */
    private SwerveModuleState[] getModuleLastDesiredStates() {
        return new SwerveModuleState[] {
            frontLeft.getLastDesiredState(),
            frontRight.getLastDesiredState(),
            backLeft.getLastDesiredState(),
            backRight.getLastDesiredState()
        };
    }

    /** useful for tuning the drivetrain current limiters to prevent brownouts */
    private double[] getModuleDriveCurrents() {
        return new double[] {
            frontLeft.getDriveMotorOutputCurrent(),
            frontRight.getDriveMotorOutputCurrent(),
            backLeft.getDriveMotorOutputCurrent(),
            backRight.getDriveMotorOutputCurrent(),
        };
    }

    private double getTotalDriveMotorCurrentDraw() {
        double totalCurrent = 0.0;
        for (double outputCurrent : getModuleDriveCurrents()) {
            totalCurrent += outputCurrent;
        }

        return totalCurrent;
    }

    private double getAverageDriveMotorCurrentDraw() {
        return getTotalDriveMotorCurrentDraw() / 4;
    }

    /**
     * gets the rotation and velocity of each module
     *
     * @return [front left, front right, back left, back right]
     */
    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
        };
    }

    /**
     * gets the rotation and distance traveled from each module
     *
     * @return [front left, front right, back left, back right]
     */
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        };
    }

    /**
     * @return robot relative ChassisSpeeds, see the wpilib coordinate system for more info
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * @return robot relative linear velocity in meters per second
     */
    public double getLinearVelocity() {
        ChassisSpeeds relativeSpeeds = getRobotRelativeSpeeds();
        return Math.hypot(relativeSpeeds.vxMetersPerSecond, relativeSpeeds.vyMetersPerSecond);
    }

    /** Method to send telemetry for robot pose data to NetworkTables */
    public void logRobotPose(Pose2d estimatedPose) {
        teleopField.setRobotPose(estimatedPose);
        autoField.setRobotPose(estimatedPose);
        Logger.recordOutput("Drive/Estimated Robot X", estimatedPose.getX());
        Logger.recordOutput("Drive/Estimated Robot Y", estimatedPose.getY());
        Logger.recordOutput(
                "Drive/Estimated Rotation", estimatedPose.getRotation().getRadians());
        Logger.recordOutput("Drive/Estimated Robot Pose", estimatedPose);
        Logger.recordOutput("Drive/Swerve Module States", getModuleStates());
        Logger.recordOutput("Drive/Linear Velocity", getLinearVelocity());
    }

    public Command nudgeForward() {
        return driveRobotRelative(() -> new ChassisSpeeds(DriveConstants.NUDGE_SPEED, 0, 0));
    }

    public Command nudgeBack() {
        return driveRobotRelative(() -> new ChassisSpeeds(-DriveConstants.NUDGE_SPEED, 0, 0));
    }

    public Command nudgeRight() {
        return driveRobotRelative(() -> new ChassisSpeeds(0, -DriveConstants.NUDGE_SPEED, 0));
    }

    public Command nudgeLeft() {
        return driveRobotRelative(() -> new ChassisSpeeds(0, DriveConstants.NUDGE_SPEED, 0));
    }
}