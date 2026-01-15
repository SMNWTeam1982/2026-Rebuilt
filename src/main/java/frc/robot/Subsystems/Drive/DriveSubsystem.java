package frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tunables.DriveBaseTunables;

/**
 * The Subsystem that the other code will interface with when interacting with the drive
 */
public class DriveSubsystem extends SubsystemBase {


    /** abstraction of the swerve module coordination */
    private final DriveBase driveBase = new DriveBase();

    /** controller for the field-relative heading of the robot */
    private final PIDController headingController = new PIDController(
        DriveBaseTunables.HEADING_P,
        DriveBaseTunables.HEADING_I,
        DriveBaseTunables.HEADING_D
    );

    /** controller for the field-relative x of the robot */
    private final PIDController xController = new PIDController(
        DriveBaseTunables.TRANSLATION_P,
        DriveBaseTunables.TRANSLATION_I,
        DriveBaseTunables.TRANSLATION_D
    );

    /** controller for the field-relative y of the robot */
    private final PIDController yController = new PIDController(
        DriveBaseTunables.TRANSLATION_P,
        DriveBaseTunables.TRANSLATION_I,
        DriveBaseTunables.TRANSLATION_D
    );

    /** set pid settings */
    public DriveSubsystem(){
        headingController.setTolerance(DriveBaseTunables.AUTO_ROTATION_TOLERANCE.getRadians());
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(DriveBaseTunables.AUTO_TRANSLATION_TOLERANCE);
        yController.setTolerance(DriveBaseTunables.AUTO_TRANSLATION_TOLERANCE);
    }

    /** update telemetry and pose estimation here */
    @Override
    public void periodic() {
        driveBase.updatePoseEstimatorOdometry();
    }

    /**
     * converts joystick inputs to field relative inputs so the robot moves relative to the drivers station it is viewed from
     * @param joystickSpeeds a chassisSpeeds with x and y being feed from the joystick values, and the theta coming from any source (usually the other stick)
     */
    public ChassisSpeeds joystickSpeedsToFieldRelativeSpeeds(ChassisSpeeds joystickSpeeds, boolean onBlueAlliance) {
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
            () -> ChassisSpeeds.fromFieldRelativeSpeeds(desiredFieldSpeeds.get(), driveBase.getHeading())
        );
    }

    /** a command that drives the robot relative to the drivers station */
    public Command driveDriversStationRelative(Supplier<ChassisSpeeds> joystickSpeeds, boolean onBlueAlliance){
        return driveFieldRelative(
            () -> joystickSpeedsToFieldRelativeSpeeds(joystickSpeeds.get(), onBlueAlliance)
        );
    }

    /**
     * drives the robot field-relative with a target rotation instead of an angular velocity
     * <p>the x & y velocities and the field rotation are relative to the field coordinate system
     * @param desiredTranslation the theta component is ignored
     */
    public Command driveTopDown(Supplier<ChassisSpeeds> desiredTranslation, Supplier<Rotation2d> desiredFieldRotation) {
        return driveFieldRelative(
            () -> {
                ChassisSpeeds fieldRelativeSpeeds = desiredTranslation.get();

                double angularVelocity = headingController.calculate(
                    driveBase.getHeading().getRadians(),
                    desiredFieldRotation.get().getRadians()
                );

                fieldRelativeSpeeds.omegaRadiansPerSecond = angularVelocity;

                return fieldRelativeSpeeds;
            }
        ).finallyDo(
            () -> {
                headingController.reset();
            }
        );
    }

    /**
     * creates a command to move the robot to a specific pose
     * <p>note that the target pose doesnt change when the command is run multiple times
     */
    public Command moveToPose(Pose2d targetPose) {
        return driveTopDown(
                () -> {
                    Pose2d currentRobotPose = driveBase.getEstimatedPose();

                    double xOutput = -xController.calculate(
                            currentRobotPose.getX(), targetPose.getX());
                    double clampedXOutput = MathUtil.clamp(
                            xOutput, -DriveBaseTunables.MAX_AUTO_SPEED, DriveBaseTunables.MAX_AUTO_SPEED);

                    double yOutput = -yController.calculate(
                            currentRobotPose.getY(), targetPose.getY());
                    double clampedYOutput = MathUtil.clamp(
                            yOutput, -DriveBaseTunables.MAX_AUTO_SPEED, DriveBaseTunables.MAX_AUTO_SPEED);

                    return new ChassisSpeeds(clampedXOutput, clampedYOutput, 0);
                },
                () -> targetPose.getRotation()
            ).until(() -> xController.atSetpoint() && yController.atSetpoint())
            .finallyDo(() -> {
                xController.reset();
                yController.reset();
            });
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
     * <p>anything that resets the pose estimator will be irrelavant with absolute position
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
}