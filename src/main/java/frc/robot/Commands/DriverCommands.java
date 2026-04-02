package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShotCalculation;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriverCommands {
    /**
     * sets the drive mode to drive and point at the supplied target
     * <p> sets the shooter's rpm calculation to the rpm for the supplied target
     * <p> this is a runOnce type command
     */
    public static Command setAimAtTarget(
            DriveSubsystem drive,
            ShooterSubsystem shooter,
            BooleanSupplier onBlueAlliance,
            Supplier<ChassisSpeeds> joystickSpeeds,
            Supplier<Translation2d> calculatedTarget,
            BooleanSupplier changeShooterRPM) {

        Supplier<ChassisSpeeds> fieldRelativeSpeeds =
                () -> DriveSubsystem.joystickSpeedsToFieldRelativeSpeeds(joystickSpeeds.get(), onBlueAlliance.getAsBoolean());
        DoubleSupplier calculatedRPM =
                () -> ShotCalculation.calculateRPM(drive.getRobotPose().getTranslation(), calculatedTarget.get());
        return drive.runOnce(
                        () -> drive.setDefaultCommand(drive.driveAndPointAtTarget(fieldRelativeSpeeds, calculatedTarget)
                                .withName("target aim")))
                .andThen(shooter.setRPMSupplier(calculatedRPM).onlyIf(changeShooterRPM))
                .withName("set target aim mode");
    }

    /** sets the shooter to its idle speed, sets the drive mode to the default mode */
    public static Command setNormalMode(
            DriveSubsystem drive,
            ShooterSubsystem shooter,
            BooleanSupplier onBlueAlliance,
            Supplier<ChassisSpeeds> joystickSpeeds,
            BooleanSupplier changeShooterRPM) {
        Supplier<ChassisSpeeds> fieldRelativeSpeeds =
                () -> DriveSubsystem.joystickSpeedsToFieldRelativeSpeeds(joystickSpeeds.get(), onBlueAlliance.getAsBoolean());
        return drive.runOnce(() -> drive.setDefaultCommand(
                        drive.driveFieldRelative(fieldRelativeSpeeds).withName("DS relative")))
                .andThen(shooter.setIdle().onlyIf(changeShooterRPM))
                .withName("set normal DS drive mode");
    }

    /** sets the shooter to idle speed, sets the robot to be driven robot-relative */
    public static Command setRobotRelativeMode(
            DriveSubsystem drive,
            ShooterSubsystem shooter,
            Supplier<ChassisSpeeds> joystickSpeeds,
            BooleanSupplier changeShooterRPM) {
        // the conversion from blue drivers station to field speeds does the same thing that controller to robot
        // relative would do
        Supplier<ChassisSpeeds> robotRelativeSpeeds =
                () -> DriveSubsystem.joystickSpeedsToFieldRelativeSpeeds(joystickSpeeds.get(), false);
        return drive.runOnce(() -> drive.setDefaultCommand(
                        drive.driveRobotRelative(robotRelativeSpeeds).withName("Robot relative")))
                .andThen(shooter.setIdle().onlyIf(changeShooterRPM))
                .withName("set robot relative drive mode");
    }

//     /** will calculate the nearest hub and then set the robot to orbit it at the current distance from it
//      * <p> sets the shooter to a calculated rpm based on the distance from the hub
//      */
//     public static Command setOrbitNearestHubAtCurrentDistance(
//             DriveSubsystem drive,
//             ShooterSubsystem shooter,
//             DoubleSupplier orbitVelocity,
//             BooleanSupplier changeShooterRPM) {
//         return drive.defer(() -> {
//                     Translation2d currentRobotTranslation = drive.getRobotPose().getTranslation();
//                     Translation2d nearestHub = ShotCalculation.getNearestHubPosition(currentRobotTranslation);
//                     DoubleSupplier calculatedRPM = () ->
//                             ShotCalculation.calculateRPM(drive.getRobotPose().getTranslation(), nearestHub);
//                     return drive.runOnce(() -> drive.setDefaultCommand(drive.orbitPoint(
//                                             orbitVelocity, nearestHub, currentRobotTranslation.getDistance(nearestHub))
//                                     .withName("orbit")))
//                             .andThen(shooter.setRPMSupplier(calculatedRPM).onlyIf(changeShooterRPM));
//                 })
//                 .withName("set orbit nearest hub");
//     }
}
