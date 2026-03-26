// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriverCommands;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Tunables.DriveBaseTunables;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Intake.StrippedIntakeSubsystem;
import frc.robot.Subsystems.Kicker.KickerSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShotCalculation;
import frc.robot.Subsystems.Vision.VisionSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    /** allows the ability to toggle the velocity Compensation
     * in the case that the velocity Compensation is not working correctly
     * you can toggle it on or off(true or false)*/
    @AutoLogOutput(key = "Driver info/velocity compensation enabled")
    private boolean velocityCompensationEnabled = true;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final VisionSubsystem vision = new VisionSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem(vision::getLastVisionResult);

    private final Supplier<Translation2d> calculatedHubTarget = () -> {
        if (velocityCompensationEnabled) {
            return ShotCalculation.getHubTarget(
                    drive.getRobotPose().getTranslation(), drive.getFieldRelativeVelocity());
        } else {
            return ShotCalculation.getNearestHubPosition(drive.getRobotPose().getTranslation());
        }
    };

    private final Supplier<Translation2d> calculatedPassTarget = () -> {
        if (velocityCompensationEnabled) {
            return ShotCalculation.getPassTarget(
                    drive.getRobotPose().getTranslation(), drive.getFieldRelativeVelocity());
        } else {
            return ShotCalculation.getNearestPassTargetPosition(
                    drive.getRobotPose().getTranslation());
        }
    };

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    // private final IntakeSubsystem intake = new IntakeSubsystem();
    private final StrippedIntakeSubsystem simpleIntake = new StrippedIntakeSubsystem();
    // private final ClimberSubsystem climber = new ClimberSubsystem();

    /** make sure that we are in the corret area for at least 1 second */
    @AutoLogOutput(key = "Driver info/robot in alliance zone")
    private final Trigger inAllianceZone = new Trigger(() -> {
                Translation2d robotPosition = drive.getRobotPose().getTranslation();
                Translation2d nearestHub = ShotCalculation.getNearestHubPosition(robotPosition);

                if (nearestHub == FieldMeasurements.BLUE_HUB_CENTER) {
                    // are we to the left of the blue hub
                    Logger.recordOutput("Driver info/nearest hub", "BLUE");
                    return robotPosition.getX() < FieldMeasurements.BLUE_HUB_CENTER.getX();
                } else if (nearestHub == FieldMeasurements.RED_HUB_CENTER) {
                    // are we to the right of the red hub
                    Logger.recordOutput("Driver info/nearest hub", "RED");
                    return robotPosition.getX() > FieldMeasurements.RED_HUB_CENTER.getX();
                } else {
                    Logger.recordOutput("Driver info/nearest hub", "no hub found");
                    return false;
                }
            })
            .debounce(1);

    /**
     * is the drive at the target heading?
     * <p> are the flywheels at the target speed?
     * <p> is the shooter in shoot mode? (is it calculating the velocity from the equations?)
     */
    @AutoLogOutput(key = "Driver info/robot ready to shoot")
    private final Trigger robotReadyToShoot = drive.atTargetHeading
            .and(shooter.velocityControllerCommands.atSetpoint)
            .and(shooter.inShootMode);

    private final boolean onBlueAlliance;

    public RobotContainer() {
        /** make sure that the robot is turned on once on the field, because this cannot change without restarting the code */
        onBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;
        CameraServer.startAutomaticCapture();
        configureDriverBindings();
        // configureOperatorBindings();

        // temporary, will not be called during comp code
        configureTestingBindings();
    }

    private void configureDriverBindings() {

        // use right/left bumbers to extend/retract the climber
        // driverController.rightBumper().debounce(0.1).whileTrue(climber.moveClimberOut());
        // driverController.leftBumper().debounce(0.1).whileTrue(climber.moveClimberIn());

        // set the drive controls to hub aim mode when pressed, and set the shooter rpm calculation
        driverController
                .a()
                .debounce(0.1)
                .onTrue(DriverCommands.setAimAtTarget(
                        drive, shooter, onBlueAlliance, this::getJoystickSpeeds, calculatedHubTarget));

        // set the drive controls to pass aim mode when pressed, and set the shooter rpm calculation
        driverController
                .x()
                .debounce(0.1)
                .onTrue(DriverCommands.setAimAtTarget(
                        drive, shooter, onBlueAlliance, this::getJoystickSpeeds, calculatedPassTarget));

        // sets the drive controls to standard field relative when pressed
        driverController
                .b()
                .debounce(0.1)
                .onTrue(DriverCommands.setNormalMode(drive, shooter, onBlueAlliance, this::getJoystickSpeeds));

        // sets the drive controls to robot relative when pressed
        driverController
                .y()
                .debounce(0.1)
                .onTrue(DriverCommands.setRobotRelativeMode(drive, shooter, this::getJoystickSpeeds));
    }

    private void configureOperatorBindings() {
        // deploy/retract the intake with a & b
        // operatorController.a().debounce(0.1).onTrue(intake.deploy());
        // operatorController.b().debounce(0.1).onTrue(intake.retract());
        operatorController.a().debounce(0.1).onTrue(simpleIntake.deploy());
        operatorController.b().debounce(0.1).onTrue(simpleIntake.stow());

        // manually start/stop the kicker
        // operatorController.rightBumper().debounce(0.1).onTrue(kicker.startKicker());
        // operatorController.leftBumper().debounce(0.1).onTrue(kicker.idleKicker());

        // automatically start/stop the kicker when the robot is ready/not ready
        // robotReadyToShoot.whileTrue(kicker.kick());

        /**
         * Disables the velocity compensation and sets the motor speed to the shooter overide speed
         */
        operatorController
                .y()
                .debounce(0.05)
                .onTrue(shooter.velocityControllerCommands
                        .setTarget(ShooterTunables.SHOOTER_OVERIDE_SPEED)
                        .andThen(Commands.runOnce(() -> {
                            velocityCompensationEnabled = false;
                        })));
        /**
         * Reenables the velocity compensation
         */
        operatorController.x().debounce(0.05).onTrue(Commands.runOnce(() -> {
            velocityCompensationEnabled = true;
        }));
    }

    private void configureTestingBindings() {
        operatorController.a().debounce(0.1).onTrue(simpleIntake.deploy());
        operatorController.b().debounce(0.1).onTrue(simpleIntake.stow());

        operatorController.x().debounce(0.1).whileTrue(simpleIntake.moveOut());
        operatorController.y().debounce(0.1).whileTrue(simpleIntake.moveIn());

        operatorController.rightBumper().debounce(0.1).onTrue(kicker.kick());
        operatorController.leftBumper().debounce(0.1).onTrue(kicker.idleKicker());

        // adjustments for testing
        operatorController
                .back()
                .debounce(0.1)
                .onTrue(shooter.velocityControllerCommands
                        .publishPIDGains()
                        .andThen(shooter.flywheelFFCommands.publishGains()));
        operatorController
                .start()
                .debounce(0.1)
                .onTrue(shooter.velocityControllerCommands
                        .updatePIDGains()
                        .andThen(shooter.flywheelFFCommands.updateGains()));

        // coarse adjustment
        operatorController.povUp().debounce(0.1).onTrue(shooter.nudgeRPM(250));

        operatorController.povDown().debounce(0.1).onTrue(shooter.nudgeRPM(-250));

        // fine adjustment
        operatorController.povRight().debounce(0.1).onTrue(shooter.nudgeRPM(1000));

        operatorController.povLeft().debounce(0.1).onTrue(shooter.nudgeRPM(-1000));
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
        return simpleIntake.deploy()
        .andThen(drive.nudgeBack().withTimeout(2)
        .andThen( DriverCommands.setAimAtTarget(
                drive, shooter, onBlueAlliance, () -> new ChassisSpeeds(), calculatedHubTarget)
        .andThen(Commands.waitUntil(robotReadyToShoot)).andThen(kicker.startKicker())));
    }
}
