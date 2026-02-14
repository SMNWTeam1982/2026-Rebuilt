package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import frc.robot.Constants.CANBus.DriveIDs;
import frc.robot.Constants.Measured.DriveBaseMeasurements;
import frc.robot.Constants.Tunables.DriveBaseTunables;
import frc.robot.PIDTools.HotPIDFTuner;
import frc.robot.Subsystems.Vision.VisionData;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * NOT A SUBSYSTEM
 *
 * <p>this class only exists so that the technical tasks of running the swerve modules wont be
 * cluttered with the functions for driving the robot
 *
 * <p>an abstraction from the swerve modules so that the Subsystem doesn't have to manage every
 * swerve module and module telemetry
 *
 * <p>this class only provides the bare-bones functionality of a swerve drive: robot-relative
 * control and odometry
 *
 * <p>this class is not designed for anything more advanced, such as pathplanner or vision
 */
public final class DriveBase {
    public final SwerveModule frontLeft =
            new SwerveModule(DriveIDs.FRONT_LEFT_DRIVE, DriveIDs.FRONT_LEFT_TURN, DriveIDs.FRONT_LEFT_ENCODER);
    public final SwerveModule frontRight =
            new SwerveModule(DriveIDs.FRONT_RIGHT_DRIVE, DriveIDs.FRONT_RIGHT_TURN, DriveIDs.FRONT_RIGHT_ENCODER);
    public final SwerveModule backLeft =
            new SwerveModule(DriveIDs.BACK_LEFT_DRIVE, DriveIDs.BACK_LEFT_TURN, DriveIDs.BACK_LEFT_ENCODER);
    public final SwerveModule backRight =
            new SwerveModule(DriveIDs.BACK_RIGHT_DRIVE, DriveIDs.BACK_RIGHT_TURN, DriveIDs.BACK_RIGHT_ENCODER);

    public final ADIS16448_IMU gyro = new ADIS16448_IMU();

    public final Supplier<Optional<VisionData>> visionResults;

    /** stores data about the swerve modules and their states */
    public final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            DriveBaseMeasurements.FRONT_LEFT_TRANSLATION,
            DriveBaseMeasurements.FRONT_RIGHT_TRANSLATION,
            DriveBaseMeasurements.BACK_LEFT_TRANSLATION,
            DriveBaseMeasurements.BACK_RIGHT_TRANSLATION);

    /** the swerve drive is initialized with a default Pose2d, (0 x, 0 y, 0 rotation) */
    public final SwerveDrivePoseEstimator swervePoseEstimator =
            new SwerveDrivePoseEstimator(driveKinematics, getGyroHeading(), getModulePositions(), new Pose2d());

    public DriveBase(Supplier<Optional<VisionData>> visionResults) {
        this.visionResults = visionResults;
    }

    /**
     * sets the pose of the pose estimator to the given pose
     *
     * <p>useful when you need to reset the pose to a given one
     *
     * <p>anything that resets the pose estimator will be irrelavant with absolute position
     */
    public void resetEstimatedPose(Pose2d newPose) {
        swervePoseEstimator.resetPosition(getGyroHeading(), getModulePositions(), newPose);
    }

    /**
     * Zeroes the heading of the pose estimator
     *
     * <p>anything that resets the pose estimator will be irrelavant with absolute position
     */
    public void zeroEstimatedHeading() {
        swervePoseEstimator.resetRotation(new Rotation2d());
    }

    /** sets all of the drivetrain motors to 0 */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * calls the update uptade method on the pose estimator, fetches the odometry data from the
     * chassis
     */
    public void updatePoseEstimatorOdometry() {
        swervePoseEstimator.update(getGyroHeading(), getModulePositions());
    }

    /** adds a vision measurment to the pose estimator, if one is available */
    public void updatePoseEstimatorVision() {
        visionResults
                .get()
                .ifPresent(visionData -> swervePoseEstimator.addVisionMeasurement(
                        visionData.pose, visionData.timestamp, visionData.standardDeviations));
    }

    /**
     * calculates the needed module states and normalizes the wheel velocities
     *
     * <p>this will automatically slow down the speed to one that is possible by the swerve drive
     * modules
     */
    public void setModulesFromRobotRelativeSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds.discretize(speeds, DriveBaseMeasurements.DRIVE_PERIOD);
        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveBaseTunables.ARTIFICIAL_MAX_SPEED);
        setModuleStates(moduleStates);
    }

    /**
     * calls setDesiredState() on each of the swerve modules
     *
     * @param desiredStates [front left, front right, back left, back right]
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** updates the turn PIDs of each of the swerve modules with new constants */
    public void updateTurnPIDs(double p, double i, double d) {
        frontLeft.updateTurnPID(p, i, d);
        frontRight.updateTurnPID(p, i, d);
        backLeft.updateTurnPID(p, i, d);
        backRight.updateTurnPID(p, i, d);
    }
    /** uses the PIDHelper to log the turn controller errors of each module */
    public void logTurnPIDErrors() {
        HotPIDFTuner.logPIDDetails("DriveBase", "FL turn controller", frontLeft.turnPIDController);
        HotPIDFTuner.logPIDDetails("DriveBase", "FR turn controller", frontRight.turnPIDController);
        HotPIDFTuner.logPIDDetails("DriveBase", "BL turn controller", backLeft.turnPIDController);
        HotPIDFTuner.logPIDDetails("DriveBase", "BR turn controller", backRight.turnPIDController);
    }

    /** logs some telemetry from each module under DriveBase/(module name) */
    public void logModuleData() {
        frontLeft.logModuleData("front left");
        frontRight.logModuleData("front right");
        backLeft.logModuleData("back left");
        backRight.logModuleData("back right");
    }

    /** Returns the heading from getEstimatedPose() */
    public Rotation2d getHeading() {
        return getEstimatedPose().getRotation();
    }

    public Rotation2d getGyroHeading(){
        return Rotation2d.fromDegrees(gyro.getGyroAngleX());
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
    public SwerveModuleState[] getModuleLastDesiredStates() {
        return new SwerveModuleState[] {
            frontLeft.getLastDesiredState(),
            frontRight.getLastDesiredState(),
            backLeft.getLastDesiredState(),
            backRight.getLastDesiredState()
        };
    }

    /** useful for tuning the drivetrain current limiters to prevent brownouts */
    public double[] getModuleDriveCurrents() {
        return new double[] {
            frontLeft.getDriveMotorOutputCurrent(),
            frontRight.getDriveMotorOutputCurrent(),
            backLeft.getDriveMotorOutputCurrent(),
            backRight.getDriveMotorOutputCurrent(),
        };
    }

    public double getTotalDriveMotorCurrentDraw() {
        double totalCurrent = 0.0;
        for (double outputCurrent : getModuleDriveCurrents()) {
            totalCurrent += outputCurrent;
        }

        return totalCurrent;
    }

    public double getAverageDriveMotorCurrentDraw() {
        return getTotalDriveMotorCurrentDraw() / 4;
    }

    /**
     * gets the rotation and velocity of each module
     *
     * @return [front left, front right, back left, back right]
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
        };
    }

    /**
     * gets the rotation and distance traveled from each module
     *
     * @return [front left, front right, back left, back right]
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        };
    }

    /**
     * @return robot relative ChassisSpeeds, see the wpilib coordinate system for more info
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return driveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * @return ChassisSpeeds relative to the field
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
    }
}
