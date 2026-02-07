// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Measured.ShooterMeasurements;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Kicker.KickerSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShotCalculation;
import frc.robot.Subsystems.Vision.VisionSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final VisionSubsystem vision = new VisionSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem(vision::getLastVisionResult);

    private final Supplier<Translation2d> shotTarget = () ->
            ShotCalculation.getShotTarget(drive.getRobotPose().getTranslation(), drive.getFieldRelativeVelocity());

    private final DoubleSupplier shotRPM = () -> {
        double targetDistance = drive.getRobotPose().getTranslation().getDistance(shotTarget.get());
        return ShooterMeasurements.hubDistanceToFlywheelRPM(targetDistance);
    };

    private final ShooterSubsystem shooter = new ShooterSubsystem(shotRPM);
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    private final Trigger confidentShot = new Trigger(() -> {
        Translation2d robotPosition = drive.getRobotPose().getTranslation();
        double distanceFromHub = robotPosition.getDistance(ShotCalculation.getNearestHubPosition(robotPosition));
        double distanceShotTarget = robotPosition.getDistance(shotTarget.get());
        return Math.abs(distanceFromHub - distanceShotTarget) < ShooterTunables.SHOOTING_POSITION_TOLERANCE;
    });

    private final boolean onBlueAlliance;

    public RobotContainer() {

        onBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;

        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {

        // set the drive controls to aim mode when pressed
        driverController
                .a()
                .debounce(0.1)
                .onTrue(drive.runOnce(() -> drive.setDefaultCommand(drive.driveAndPointAtTarget(
                                        () -> DriveSubsystem.joystickSpeedsToFieldRelativeSpeeds(
                                                getJoystickSpeeds(), onBlueAlliance),
                                        shotTarget::get)
                                .withName("Hub aim")))
                        .withName("set auto aim mode"));

        // sets the drive controls to standard field relative when pressed
        driverController
                .b()
                .debounce(0.1)
                .onTrue(drive.runOnce(() -> drive.setDefaultCommand(
                                drive.driveFieldRelative(() -> DriveSubsystem.joystickSpeedsToFieldRelativeSpeeds(
                                                getJoystickSpeeds(), onBlueAlliance))
                                        .withName("DS relative")))
                        .withName("set DS relative mode"));

        // sets the drive controls to robot relative when pressed
        driverController
                .x()
                .debounce(0.1)
                .onTrue(drive.runOnce(() -> drive.setDefaultCommand(drive.driveRobotRelative(this::getJoystickSpeeds)
                                .withName("robot relative")))
                        .withName("set robot relative mode"));
    }

    private void configureOperatorBindings() {
        operatorController.a().debounce(0.1).onTrue(intake.deploy());
        operatorController.b().debounce(0.1).onTrue(intake.retract());

        Trigger robotReadyToShoot =
                drive.atTargetHeading.and(shooter.flywheelsUpToSpeed).and(shooter.inShootMode);

        // manually start the kicker
        operatorController.rightBumper().debounce(0.1).onTrue(kicker.startKicker());
        // right trigger starts shooting
        operatorController.rightTrigger().debounce(0.1).and(robotReadyToShoot).onTrue(kicker.startKicker());
        // left trigger stops shooting
        operatorController.leftTrigger().debounce(0.1).onTrue(kicker.stopKicker());

        // start spinning up the flywheels
        operatorController.y().debounce(0.1).onTrue(shooter.setShootMode());

        // set the flywheels to an idle speed
        operatorController.x().debounce(0.1).onTrue(shooter.setIdle());

        // adjustments for testing

        operatorController.start().debounce(0.1).onTrue(shooter.updateVelocityPIDs());
        operatorController.back().debounce(0.1).onTrue(shooter.publishVelocityGains());

        // coarse adjustment
        operatorController.povUp().debounce(0.1).onTrue(shooter.changeHeldRPM(250));

        operatorController.povDown().debounce(0.1).onTrue(shooter.changeHeldRPM(-250));

        // fine adjustment
        operatorController.povRight().debounce(0.1).onTrue(shooter.changeHeldRPM(50));

        operatorController.povLeft().debounce(0.1).onTrue(shooter.changeHeldRPM(-50));
    }

    private ChassisSpeeds getJoystickSpeeds() {
        return new ChassisSpeeds(
                driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
