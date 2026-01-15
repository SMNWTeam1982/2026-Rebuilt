package frc.robot.Subsystems.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Tunables.DriveBaseTunables;
import frc.robot.Subsystems.Vision.VisionData;

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

    public DriveSubsystem(){}

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
}