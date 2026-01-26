// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Measured.ShooterMeasurements;
import frc.robot.Constants.Tunables.FieldTunables;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Kicker.KickerSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShotCalculation;
import java.util.Optional;
import java.util.function.Supplier;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final DriveSubsystem drive = new DriveSubsystem(() -> Optional.empty());
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();

    private final boolean onBlueAlliance;
    private final Translation2d hubCenter;

    private final Supplier<Translation2d> shotTarget;

    private final Trigger confidentShot;

    public RobotContainer() {

        onBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;

        if (onBlueAlliance) {
            hubCenter = FieldMeasurements.BLUE_HUB_CENTER;
        } else {
            hubCenter = FieldMeasurements.RED_HUB_CENTER;
        }

        confidentShot = new Trigger(() -> {
            Pose2d robotPose = drive.getRobotPose();
            Translation2d newHubPosition = ShotCalculation.calculateScoringTarget(
                    robotPose.getTranslation(), hubCenter, drive.getFieldRelativeVelocity());

            double futureDistanceFromHub = newHubPosition.getDistance(robotPose.getTranslation());
            double currentDistanecFromHub = robotPose.getTranslation().getDistance(hubCenter);

            // if the distance from the hub changes by a negligable amount in the future, then the shot confidence is
            // high
            return Math.abs(futureDistanceFromHub - currentDistanecFromHub)
                    <= ShooterTunables.SHOOTING_POSITION_TOLERANCE;
        });

        shotTarget = () -> ShotCalculation.calculateScoringTargetMultiStep(
                drive.getRobotPose().getTranslation(),
                hubCenter,
                drive.getFieldRelativeVelocity(),
                ShooterTunables.SHOT_PREDICTION_ITERATIONS);

        shooter.setDefaultCommand(shooter.runAtRPM(() -> ShooterMeasurements.hubDistanceToFlywheelRPM(
                drive.getRobotPose().getTranslation().getDistance(shotTarget.get()))));

        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {

        // set the drive controls to aim mode when pressed
        driverController
                .a()
                .debounce(0.1)
                .onTrue(drive.runOnce(() -> drive.setDefaultCommand(drive.driveAndPointAtTarget(
                        () -> DriveSubsystem.joystickSpeedsToFieldRelativeSpeeds(getJoystickSpeeds(), onBlueAlliance),
                        shotTarget::get))));

        // sets the drive controls to standard field relative when pressed
        driverController
                .b()
                .debounce(0.1)
                .onTrue(drive.runOnce(() -> drive.setDefaultCommand(drive.driveFieldRelative(() ->
                        DriveSubsystem.joystickSpeedsToFieldRelativeSpeeds(getJoystickSpeeds(), onBlueAlliance)))));

        // sets the drive controls to robot relative when pressed
        driverController
                .x()
                .debounce(0.1)
                .onTrue(drive.runOnce(
                        () -> drive.setDefaultCommand(drive.driveRobotRelative(this::getJoystickSpeeds))));

        // a backup control method that moves the robot to a certain distance from the hub
        driverController
                .y()
                .debounce(0.1)
                .onTrue(drive.runOnce(() -> drive.setDefaultCommand(drive.orbitPoint(
                        () -> -driverController.getLeftY(), hubCenter, FieldTunables.HUB_SCORING_DISTANCE))));
    }

    private void configureOperatorBindings() {
        operatorController.a().debounce(0.1).onTrue(intake.deploy());
        operatorController.b().debounce(0.1).onTrue(intake.retract());

        Trigger robotReadyToShoot = drive.atTargetHeading.and(shooter.flywheelUpToSpeed);

        // right trigger starts shooting
        operatorController.rightTrigger().debounce(0.1).and(robotReadyToShoot).onTrue(kicker.startKicker());
        // left trigger stops shooting
        operatorController.leftTrigger().debounce(0.1).onTrue(kicker.stopKicker());
    }

    private ChassisSpeeds getJoystickSpeeds() {
        return new ChassisSpeeds(
                driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
