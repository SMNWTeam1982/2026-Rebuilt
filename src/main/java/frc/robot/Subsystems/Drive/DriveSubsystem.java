package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Measured.PathplannerMeasurements;
import frc.robot.Constants.Tunables.DriveBaseTunables;
import frc.robot.PIDTools.HotPIDTuner;
import frc.robot.Subsystems.Vision.VisionData;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** The Subsystem that the other code will interface with when interacting with the drive */
public class DriveSubsystem extends SubsystemBase {

    /** abstraction of the swerve module coordination */
    private final DriveBase driveBase;

    /** controller for the field-relative heading of the robot */
    private final PIDController headingController =
            new PIDController(DriveBaseTunables.HEADING_P, DriveBaseTunables.HEADING_I, DriveBaseTunables.HEADING_D);

    public final Trigger atTargetHeading = new Trigger(headingController::atSetpoint);

    /** controller for the field-relative x of the robot */
    private final PIDController xController = new PIDController(
            DriveBaseTunables.TRANSLATION_P, DriveBaseTunables.TRANSLATION_I, DriveBaseTunables.TRANSLATION_D);

    /** controller for the field-relative y of the robot */
    private final PIDController yController = new PIDController(
            DriveBaseTunables.TRANSLATION_P, DriveBaseTunables.TRANSLATION_I, DriveBaseTunables.TRANSLATION_D);

    public final Trigger atTargetTranslation = new Trigger(xController::atSetpoint).and(yController::atSetpoint);

    /** set pid settings */
    public DriveSubsystem(Supplier<Optional<VisionData>> visionResults) {
        driveBase = new DriveBase(visionResults);

        headingController.setTolerance(DriveBaseTunables.AUTO_ROTATION_TOLERANCE.getRadians());
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(DriveBaseTunables.AUTO_TRANSLATION_TOLERANCE);
        yController.setTolerance(DriveBaseTunables.AUTO_TRANSLATION_TOLERANCE);

        /** configure last auto build  */
        AutoBuilder.configure(
                this::getRobotPose, // robot pose supplier
                driveBase::resetEstimatedPose, // drivebase function
                driveBase::getRobotRelativeSpeeds, // drivebase function
                (speeds, feedforwards) -> driveBase.setModulesFromRobotRelativeSpeeds(speeds), //
                new PPHolonomicDriveController(
                        new PIDConstants(
                                DriveBaseTunables.TRANSLATION_P,
                                DriveBaseTunables.TRANSLATION_I,
                                DriveBaseTunables.TRANSLATION_D),
                        new PIDConstants(
                                DriveBaseTunables.HEADING_P, DriveBaseTunables.HEADING_I, DriveBaseTunables.HEADING_D)),
                PathplannerMeasurements.PATHPLANNER_CONFIG, // robot config
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red; //
                    }
                    return false;
                },
                this // reference to this subsytem to set requirements
                );
    }

    /** update telemetry and pose estimation here */
    @Override
    public void periodic() {
        driveBase.updatePoseEstimatorOdometry();
        driveBase.updatePoseEstimatorVision();
        driveBase.logModuleData();
        Logger.recordOutput("Drive/Field Reletive Velocity", getFieldRelativeVelocity());
        Logger.recordOutput("Drive/Robot Pose", getRobotPose());
        if (getCurrentCommand() == null) {
            Logger.recordOutput("Drive/drive command", "no active command");
        } else {
            Logger.recordOutput("Drive/drive command", this.getCurrentCommand().getName());
        }
        HotPIDTuner.logPIDErrors("Drive", "heading controller", headingController);
        HotPIDTuner.logPIDErrors("Drive", "x translation controller", xController);
        HotPIDTuner.logPIDErrors("Drive", "y translation controller", yController);
        driveBase.logTurnPIDErrors();
    }

    /**
     * converts joystick inputs to field relative inputs so the robot moves relative to the drivers
     * station it is viewed from
     *
     * @param joystickSpeeds a chassisSpeeds with x and y being feed from the joystick values, and the
     *     theta coming from any source (usually the other stick)
     */
    public static ChassisSpeeds joystickSpeedsToFieldRelativeSpeeds(
            ChassisSpeeds joystickSpeeds, boolean onBlueAlliance) {
        if (onBlueAlliance) {
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

    /** drives the robot with chassis speeds relative to the robot coordinate system */
    public Command driveRobotRelative(Supplier<ChassisSpeeds> desiredRobotSpeeds) {
        return run(() -> {
            driveBase.setModulesFromRobotRelativeSpeeds(desiredRobotSpeeds.get());
        });
    }

    /** drives the robot with chassis speeds relative to the field coordinate system */
    public Command driveFieldRelative(Supplier<ChassisSpeeds> desiredFieldSpeeds) {
        return driveRobotRelative(
                () -> ChassisSpeeds.fromFieldRelativeSpeeds(desiredFieldSpeeds.get(), driveBase.getHeading()));
    }

    /**
     * drives the robot field-relative with a target rotation instead of an angular velocity
     *
     * <p>the x & y velocities and the field rotation are relative to the field coordinate system
     *
     * @param fieldRelativeTranlations the theta component is ignored
     */
    public Command driveTopDown(
            Supplier<ChassisSpeeds> fieldRelativeTranlations, Supplier<Rotation2d> desiredFieldRotation) {
        return driveFieldRelative(() -> {
                    ChassisSpeeds fieldRelativeSpeeds = fieldRelativeTranlations.get();

                    double angularVelocity = headingController.calculate(
                            driveBase.getHeading().getRadians(),
                            desiredFieldRotation.get().getRadians());

                    fieldRelativeSpeeds.omegaRadiansPerSecond = angularVelocity;

                    return fieldRelativeSpeeds;
                })
                .finallyDo(() -> {
                    headingController.reset();
                });
    }

    /**
     * this command allows the robot to be translated while it points at a target
     *
     * @param fieldRelativeSpeeds the theta component is ignored
     */
    public Command driveAndPointAtTarget(Supplier<ChassisSpeeds> fieldRelativeSpeeds, Supplier<Translation2d> target) {
        return driveTopDown(
                fieldRelativeSpeeds, target.get().minus(getRobotPose().getTranslation())::getAngle);
    }

    /** drives the robot in a circle around the orbitCenter with a radius of the orbitDistance */
    public Command orbitPoint(Supplier<Double> orbitDirection, Translation2d orbitCenter, double orbitDistance) {
        return driveAndPointAtTarget(
                () -> {
                    Pose2d currentRobotPose = driveBase.getEstimatedPose();

                    double distanceControllerOutput = xController.calculate(
                            currentRobotPose.getTranslation().getDistance(orbitCenter), orbitDistance);

                    // the x coordinate will be rotated to move the robot away and towards the hub
                    // the y coordinate will be rotated to move the robot left and right relative to the
                    // center of the hub
                    return ChassisSpeeds.fromRobotRelativeSpeeds(
                            new ChassisSpeeds(distanceControllerOutput, orbitDirection.get(), 0.0),
                            orbitCenter.minus(currentRobotPose.getTranslation()).getAngle());
                },
                () -> orbitCenter);
    }

    /**
     * creates a command that moves the robot towards an updatable pose
     *
     * <p>will reset the position and rotation PIDs when it gets ended
     */
    public Command trackPose(Supplier<Pose2d> targetPose) {
        return driveFieldRelative(() -> {
                    Pose2d currentRobotPose = driveBase.getEstimatedPose();
                    Pose2d currentTargetPose = targetPose.get();

                    double angularVelocity = headingController.calculate(
                            currentRobotPose.getRotation().getRadians(),
                            currentTargetPose.getRotation().getRadians());

                    double xOutput = -xController.calculate(currentRobotPose.getX(), currentTargetPose.getX());
                    double clampedXOutput = MathUtil.clamp(
                            xOutput, -DriveBaseTunables.MAX_AUTO_SPEED, DriveBaseTunables.MAX_AUTO_SPEED);

                    double yOutput = -yController.calculate(currentRobotPose.getY(), currentTargetPose.getY());
                    double clampedYOutput = MathUtil.clamp(
                            yOutput, -DriveBaseTunables.MAX_AUTO_SPEED, DriveBaseTunables.MAX_AUTO_SPEED);

                    return new ChassisSpeeds(clampedXOutput, clampedYOutput, angularVelocity);
                })
                .finallyDo(() -> {
                    headingController.reset();
                    xController.reset();
                    yController.reset();
                });
    }

    /**
     * creates a command to move the robot to a specific pose
     *
     * <p>note that the target pose doesnt change when the command is run multiple times
     */
    public Command moveToPose(Pose2d targetPose) {
        return trackPose(() -> targetPose)
                .until(() -> xController.atSetpoint() && yController.atSetpoint() && headingController.atSetpoint());
    }

    public Command nudgeForward() {
        return driveRobotRelative(() -> new ChassisSpeeds(DriveBaseTunables.NUDGE_SPEED, 0, 0));
    }

    public Command nudgeBack() {
        return driveRobotRelative(() -> new ChassisSpeeds(-DriveBaseTunables.NUDGE_SPEED, 0, 0));
    }

    public Command nudgeRight() {
        return driveRobotRelative(() -> new ChassisSpeeds(0, -DriveBaseTunables.NUDGE_SPEED, 0));
    }

    public Command nudgeLeft() {
        return driveRobotRelative(() -> new ChassisSpeeds(0, DriveBaseTunables.NUDGE_SPEED, 0));
    }

    public Pose2d getRobotPose() {
        return driveBase.getEstimatedPose();
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return driveBase.getFieldRelativeSpeeds();
    }

    /**
     * Resets the pose estimator to the specified pose.
     *
     * <p>this will be useful for the questnav because as of 15 sep 2025 questnav is relative
     *
     * <p>this will be irrelavant if we are using absolute position
     */
    public Command resetEstimatedPose(Pose2d pose) {
        return runOnce(() -> driveBase.resetEstimatedPose(pose));
    }

    /**
     * Zeroes the heading of the pose estimator
     *
     * <p>anything that resets the pose estimator will be overridden by absolute position from vision
     */
    public Command zeroEstimatedHeading() {
        return runOnce(driveBase::zeroEstimatedHeading);
    }

    /** sets all of the drivetrain motors to 0 */
    public Command stop() {
        return runOnce(driveBase::stop);
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
                driveBase.frontLeft.runMotors(driveAmount.getAsDouble(), turnAmount.getAsDouble());
            }
            if (moduleIndex == 1) {
                driveBase.frontRight.runMotors(driveAmount.getAsDouble(), turnAmount.getAsDouble());
            }
            if (moduleIndex == 2) {
                driveBase.backLeft.runMotors(driveAmount.getAsDouble(), turnAmount.getAsDouble());
            }
            if (moduleIndex == 3) {
                driveBase.backRight.runMotors(driveAmount.getAsDouble(), turnAmount.getAsDouble());
            }
        });
    }

    /** makes a command to change the pid values to the ones provided, you should defer this if you want to change this multiple times */
    public Command setWheelTurnPIDs(double p, double i, double d) {
        return runOnce(() -> {
            driveBase.updateTurnPIDs(p, i, d);
        });
    }

    /** makes a command to update the heading pid to the provided values */
    public Command setHeadingPID(double p, double i, double d) {
        return runOnce(() -> {
            headingController.setPID(p, i, d);
        });
    }

    /** makes a command to update the translation pids to the provided values */
    public Command setTranslationPIDs(double p, double i, double d) {
        return runOnce(() -> {
            xController.setPID(p, i, d);
            yController.setPID(p, i, d);
        });
    }

    public Command publishHeadingGains() {
        // x & y controllers should have the same gains
        return HotPIDTuner.publishGainsToNetworkTables(this, headingController);
    }

    public Command updateHeadingPID() {
        return HotPIDTuner.setGainsFromNetworkTables(this, headingController);
    }

    public Command publishTranslationGains() {
        // x & y controllers should have the same gains
        return HotPIDTuner.publishGainsToNetworkTables(this, xController);
    }

    public Command updateTranslationPIDs() {
        return HotPIDTuner.setGainsFromNetworkTables(this, xController, yController);
    }

    public Command publishModuleTurnGains() {
        // all modules should have the same gains
        return HotPIDTuner.publishGainsToNetworkTables(this, driveBase.frontLeft.turnPIDController);
    }

    public Command updateModuleTurnGains() {
        return HotPIDTuner.setGainsFromNetworkTables(
                this,
                driveBase.frontRight.turnPIDController,
                driveBase.frontLeft.turnPIDController,
                driveBase.backRight.turnPIDController,
                driveBase.backLeft.turnPIDController);
    }
}
