// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.Tunables.DriveBaseTunables;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
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
    private final ClimberSubsystem climber = new ClimberSubsystem();

    private final Trigger confidentShot = new Trigger(() -> {
        Translation2d robotPosition = drive.getRobotPose().getTranslation();
        double distanceFromHub = robotPosition.getDistance(ShotCalculation.getNearestHubPosition(robotPosition));
        double distanceShotTarget = robotPosition.getDistance(shotTarget.get());
        return Math.abs(distanceFromHub - distanceShotTarget) < ShooterTunables.SHOOTING_POSITION_TOLERANCE;
    });

    /** make sure that we are in the corret area for at least 1 second */
    private final Trigger inShootingArea = new Trigger(() -> {
                Translation2d robotPosition = drive.getRobotPose().getTranslation();
                Translation2d nearestHub = ShotCalculation.getNearestHubPosition(robotPosition);

                if (nearestHub == FieldMeasurements.BLUE_HUB_CENTER) {
                    // are we to the left of the blue hub
                    return robotPosition.getX() < FieldMeasurements.BLUE_HUB_CENTER.getX();
                } else {
                    // are we to the right of the red hub
                    return robotPosition.getX() > FieldMeasurements.RED_HUB_CENTER.getX();
                }
            })
            .debounce(1);

    /**
     * is the drive at the target heading?
     * <p> are the flywheels at the target speed?
     * <p> is the shooter in shoot mode? (is it calculating the velocity from the equations?)
     * <p> are we in the zone that we are allowed to shoot in?
     */
    private final Trigger robotReadyToShoot = drive.atTargetHeading
            .and(shooter.velocityControllerCommands.atSetpoint)
            .and(shooter.inShootMode)
            .and(inShootingArea);

    private final boolean onBlueAlliance;

    public RobotContainer() {
        /** make sure that the robot is turned on once on the field, because this cannot change without restarting the code */
        onBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;

        configureDriverBindings();
        configureOperatorBindings();

        // temporary, will not be called during comp code
        configureTestingBindings();
    }

    private void configureDriverBindings() {

        // use right/left bumbers to extend/retract the climber
        driverController.rightBumper().debounce(0.1).whileTrue(climber.moveClimberOut());
        driverController.leftBumper().debounce(0.1).whileTrue(climber.moveClimberIn());

        // set the drive controls to aim mode when pressed, and set the shooter to shoot mode (spin up)
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
        // deploy/retract the intake with a & b
        operatorController.a().debounce(0.1).onTrue(intake.deploy());
        operatorController.b().debounce(0.1).onTrue(intake.retract());

        // manually start/stop the kicker
        operatorController.rightBumper().debounce(0.1).onTrue(kicker.startKicker());
        operatorController.leftBumper().debounce(0.1).onTrue(kicker.stopKicker());

        // set the flywheels to spin up when right trigger is pressed
        operatorController.rightTrigger().onTrue(shooter.setShootMode());

        // set the flywheels to their idle speed
        operatorController.leftTrigger().onTrue(shooter.setIdle());

        // automatically start/stop the kicker when the robot is ready/not ready
        robotReadyToShoot.whileTrue(kicker.kick());
    }

    private void configureTestingBindings() {
        // adjustments for testing
        operatorController.back().debounce(0.1).onTrue(shooter.velocityControllerCommands.publishPIDGains());
        operatorController.start().debounce(0.1).onTrue(shooter.velocityControllerCommands.updatePIDGains());

        // coarse adjustment
        operatorController.povUp().debounce(0.1).onTrue(shooter.nudgeRPM(250));

        operatorController.povDown().debounce(0.1).onTrue(shooter.nudgeRPM(-250));

        // fine adjustment
        operatorController.povRight().debounce(0.1).onTrue(shooter.nudgeRPM(50));

        operatorController.povLeft().debounce(0.1).onTrue(shooter.nudgeRPM(-50));
    }

    private ChassisSpeeds getJoystickSpeeds() {
        return new ChassisSpeeds(
                MathUtil.applyDeadband(
                        driverController.getLeftX(), DriveBaseTunables.INPUT_DEADZONE, DriveBaseTunables.DRIVE_SPEED),
                MathUtil.applyDeadband(
                        driverController.getLeftY(), DriveBaseTunables.INPUT_DEADZONE, DriveBaseTunables.DRIVE_SPEED),
                MathUtil.applyDeadband(
                        driverController.getRightX(), DriveBaseTunables.INPUT_DEADZONE, DriveBaseTunables.TURN_SPEED));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
