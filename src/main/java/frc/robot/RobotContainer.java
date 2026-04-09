// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutoCommands;
import frc.robot.Commands.DriverCommands;
import frc.robot.Commands.RobotCommands;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Tunables.DriveBaseTunables;
import frc.robot.Constants.Tunables.KickerTunables;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Kicker.KickerSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShotCalculation;
import frc.robot.Subsystems.Vision.VisionSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    @AutoLogOutput(key = "Driver info/onBlueAlliance")
    private final BooleanSupplier onBlueAlliance =
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

    /** allows the ability to toggle the velocity Compensation
     * in the case that the velocity Compensation is not working correctly
     * you can toggle it on or off(true or false)*/
    @AutoLogOutput(key = "Driver info/velocity compensation enabled")
    private boolean velocityCompensationEnabled = false;

    /**
     * controls if the set drive mode commands triggered by the driver will also change the shooter RPM
     */
    @AutoLogOutput(key = "Driver info/driver can change shooter RPM")
    private boolean driverCanChangeShooterRPM = true;

    @AutoLogOutput(key = "Driver info/auto kick enabled")
    private boolean autoKickerModeEnabled = true;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final LoggedDashboardChooser<Command> autoChooser;

    // these suppliers are so that the status of the right trigger can be logged
    // if the robot is in a good spot for reliable shooting at the given rpm, the driver or operator can press the right
    // trigger to leave a marker in the log
    // this marker can be looked at later to make it easier to find data points for the distance->RPM function
    @AutoLogOutput(key = "Driver info/operator says at good shooting position")
    private final BooleanSupplier operatorSaysAtGoodShootingPosition =
            operatorController.rightTrigger().debounce(0.05)::getAsBoolean;

    @AutoLogOutput(key = "Driver info/driver says at good shooting position")
    private final BooleanSupplier driverSaysAtGoodShootingPosition =
            driverController.rightTrigger().debounce(0.05)::getAsBoolean;

    private final VisionSubsystem vision = new VisionSubsystem();
    private final DriveSubsystem drive = new DriveSubsystem(vision::getLastVisionResult, onBlueAlliance);

    // @AutoLogOutput(key = "Driver info/calculated hub target")
    private final Supplier<Translation2d> calculatedHubTarget = () -> {
        if (velocityCompensationEnabled) {
            var buf = ShotCalculation.getHubTarget(
                    drive.getRobotPose().getTranslation(),
                    drive.getFieldRelativeVelocity(),
                    onBlueAlliance.getAsBoolean());
            Logger.recordOutput("Driver info/calculated hub target", buf);
            return buf;
        } else {
            var buf = ShotCalculation.getAllianceHubPosition(onBlueAlliance.getAsBoolean());
            Logger.recordOutput("Driver info/calculated hub target", buf);
            return buf;
        }
    };

    // @AutoLogOutput(key = "Driver info/calculated pass target")
    private final Supplier<Translation2d> calculatedPassTarget = () -> {
        if (velocityCompensationEnabled) {
            return ShotCalculation.getPassTarget(
                    drive.getRobotPose().getTranslation(),
                    drive.getFieldRelativeVelocity(),
                    onBlueAlliance.getAsBoolean());
        } else {
            return ShotCalculation.getNearestAlliancePassTarget(
                    drive.getRobotPose().getTranslation(), onBlueAlliance.getAsBoolean());
        }
    };

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    /** make sure that we are in the correct area for at least 1 second */
    @AutoLogOutput(key = "Driver info/robot in alliance zone") // autologging this trigger should call it every period
    private final Trigger inAllianceZone = new Trigger(() -> {
                Translation2d robotPosition = drive.getRobotPose().getTranslation();
                Translation2d nearestHub = ShotCalculation.getAllianceHubPosition(onBlueAlliance.getAsBoolean());
                Logger.recordOutput("Driver info/distance from nearest hub", robotPosition.getDistance(nearestHub));

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
            .and(shooter.readyToShoot)
            .and(shooter.inShootMode)
            .and(() -> drive.getLinearSpeed() <= KickerTunables.ROBOT_MAX_SPEED_WHEN_KICKING);

    public RobotContainer() {
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(1);

        // automatically disable the vision LED mode when teleOp is enabled
        // robotEnabled.onTrue(vision.deactivateLEDMode());
        // automatically disable the vision LED mode when teleOp is enabled
        // robotEnabled.onFalse(vision.activateLEDMode());

        configureDriverBindings();
        configureOperatorBindings();

        addNamedCommands();

        autoChooser = new LoggedDashboardChooser<Command>("auto chooser", AutoBuilder.buildAutoChooser());

        // temporary, will not be called during comp code
        // configureTestingBindings();
    }

    private void addNamedCommands() {
        // auto commands
        NamedCommands.registerCommand(
                "hub shooting procedure 3 seconds",
                AutoCommands.shootIntoHub(drive, shooter, kicker, Seconds.of(3), onBlueAlliance));
        NamedCommands.registerCommand(
                "hub shooting procedure 5 seconds",
                AutoCommands.shootIntoHub(drive, shooter, kicker, Seconds.of(5), onBlueAlliance));
        NamedCommands.registerCommand(
                "hub shooting procedure 10 seconds",
                AutoCommands.shootIntoHub(drive, shooter, kicker, Seconds.of(10), onBlueAlliance));

        NamedCommands.registerCommand("stop and deploy intake", AutoCommands.deployIntake(drive, intake));

        NamedCommands.registerCommand(
                "shoot & kick",
                shooter.setTarget(() -> drive.getRobotPose().getTranslation(), calculatedHubTarget)
                        .andThen(shooter.runPIDs().withTimeout(ShooterTunables.AUTO_SPIN_UP_TIME))
                        .andThen(kicker.kick().alongWith(shooter.runPIDs())));

        // shooter
        NamedCommands.registerCommand(
                "set shooter to target the hub",
                shooter.setTarget(drive.getRobotPose()::getTranslation, calculatedHubTarget)
                        .asProxy());
        NamedCommands.registerCommand(
                "spin up shooter",
                shooter.setTarget(drive.getRobotPose()::getTranslation, calculatedHubTarget)
                        .asProxy()
                        .andThen(new WaitCommand(ShooterTunables.AUTO_SPIN_UP_TIME)));
        NamedCommands.registerCommand("idle shooter", shooter.setIdle().asProxy());
        // kicker
        NamedCommands.registerCommand("kick", kicker.kick());
        NamedCommands.registerCommand("kick 5 seconds", kicker.kick().withTimeout(5));
        NamedCommands.registerCommand("kick 7.5 seconds", kicker.kick().withTimeout(7.5));
        NamedCommands.registerCommand("kick 10 seconds", kicker.kick().withTimeout(10));
        NamedCommands.registerCommand("start kicker", kicker.setHigh());
        NamedCommands.registerCommand("stop kicker", kicker.setIdle());
        // intake
        NamedCommands.registerCommand("start intaking", intake.startIntaking());
        NamedCommands.registerCommand("stop intaking", intake.stopIntaking());
        NamedCommands.registerCommand("deploy intake", intake.deploy());
        NamedCommands.registerCommand("stow intake", intake.stow());
        // drive
        NamedCommands.registerCommand("stop drive", drive.stop());
    }

    private Command enterManualMode() {
        return Commands.runOnce(() -> {
            autoKickerModeEnabled = false;
            driverCanChangeShooterRPM = false;
        });
    }

    private Command exitManualMode() {
        return Commands.runOnce(() -> {
            autoKickerModeEnabled = true;
            driverCanChangeShooterRPM = true;
        });
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
                        drive,
                        shooter,
                        onBlueAlliance,
                        this::getJoystickSpeeds,
                        calculatedHubTarget,
                        () -> driverCanChangeShooterRPM));

        // set the drive controls to pass aim mode when pressed, and set the shooter rpm calculation
        driverController
                .x()
                .debounce(0.1)
                .onTrue(DriverCommands.setAimAtTarget(
                        drive,
                        shooter,
                        onBlueAlliance,
                        this::getJoystickSpeeds,
                        calculatedPassTarget,
                        () -> driverCanChangeShooterRPM));

        // sets the drive controls to standard field relative when pressed
        driverController
                .b()
                .debounce(0.1)
                .onTrue(DriverCommands.setNormalMode(
                        drive, shooter, onBlueAlliance, this::getJoystickSpeeds, () -> driverCanChangeShooterRPM));

        // set the drive controls to pass aim mode when pressed, and set the shooter rpm calculation
        driverController
                .y()
                .debounce(0.1)
                .onTrue(DriverCommands.setAimAtTarget(
                        drive,
                        shooter,
                        onBlueAlliance,
                        this::getJoystickSpeeds,
                        () -> ShotCalculation.getNearestNeutralZonePassTarget(
                                drive.getRobotPose().getTranslation()),
                        () -> driverCanChangeShooterRPM));

        // sets the drive controls to robot relative when pressed
        driverController
                .povUp()
                .debounce(0.1)
                .onTrue(DriverCommands.setRobotRelativeMode(
                        drive, shooter, this::getJoystickSpeeds, () -> driverCanChangeShooterRPM));

        // sets the drive mode to hub orbit when pressed
        // driverController
        //         .povUp()
        //         .debounce(0.1)
        //         .onTrue(DriverCommands.setOrbitNearestHubAtCurrentDistance(
        //                 drive, shooter, driverController::getLeftX, () -> driverCanChangeShooterRPM));
    }

    private void configureOperatorBindings() {

        // operatorController.a().onTrue(shooter.turnOff().andThen(kicker.turnOff()));

        operatorController
                .rightTrigger()
                .and(operatorController.leftTrigger())
                .debounce(2.0)
                .onTrue(RobotCommands.tryUnjam(shooter, kicker, intake));

        // deploy/retract the intake with a & b
        operatorController.a().debounce(0.05).whileTrue(intake.startIntaking().andThen(intake.moveOut()));
        operatorController.b().debounce(0.05).whileTrue(intake.stopIntaking().andThen(intake.moveIn()));

        operatorController.rightTrigger().debounce(0.05).whileTrue(intake.moveIn());

        // manually start/stop the kicker
        operatorController
                .rightBumper()
                .debounce(0.05)
                .onTrue(Commands.runOnce(() -> {
                            autoKickerModeEnabled = false;
                        })
                        .andThen(kicker.kick()));

        operatorController
                .leftBumper()
                .debounce(0.05)
                .onTrue(Commands.runOnce(() -> {
                            autoKickerModeEnabled = false;
                        })
                        .andThen(kicker.setIdle()));

        // automatically start/stop the kicker when the robot is ready/not ready
        robotReadyToShoot.and(() -> autoKickerModeEnabled).whileTrue(kicker.kick());

        operatorController.x().onTrue(exitManualMode());

        operatorController.y().onTrue(enterManualMode());

        // /**
        //  * Disables the velocity compensation
        //  */
        // operatorController.back().debounce(0.05).onTrue(Commands.runOnce(() -> {
        //     velocityCompensationEnabled = false;
        // }));

        // /**
        //  * Reenables the velocity compensation
        //  */
        // operatorController.start().debounce(0.05).onTrue(Commands.runOnce(() -> {
        //     velocityCompensationEnabled = true;
        // }));

        // the shooter's rpm WILL be set when the driver changes mode
        // operatorController.x().debounce(0.05).onTrue(Commands.runOnce(() -> {
        //     driverCanChangeShooterRPM = true;
        // }));

        // // the shooter's rpm will NOT be set when the driver changes mode
        // operatorController.y().debounce(0.05).onTrue(Commands.runOnce(() -> {
        //     driverCanChangeShooterRPM = false;
        // }));

        // enable defense mode
        operatorController
                .back()
                .debounce(.05)
                .onTrue(RobotCommands.enterDefenseMode(shooter, kicker, intake).andThen(enterManualMode()));

        // Disable defense mode
        operatorController
                .start()
                .debounce(.05)
                .onTrue(RobotCommands.exitDefenseMode(shooter, kicker, intake).andThen(exitManualMode()));

        // Toggle Auto kicker
        // CHANGE BUTTON
        // operatorController.leftTrigger().debounce(.05).onTrue(Commands.runOnce(() -> {
        //     autoKickerModeEnabled = true;
        // }));

        // speed overides for shooter
        operatorController
                .povRight()
                .debounce(0.05)
                .onTrue(shooter.velocityControllerCommands.setTarget(ShooterTunables.SPEED_OVERRIDE_1));
        operatorController
                .povDown()
                .debounce(0.05)
                .onTrue(shooter.velocityControllerCommands.setTarget(ShooterTunables.SPEED_OVERRIDE_2));
        operatorController
                .povLeft()
                .debounce(0.05)
                .onTrue(shooter.velocityControllerCommands.setTarget(ShooterTunables.SPEED_OVERRIDE_3));
        operatorController
                .povUp()
                .debounce(0.05)
                .onTrue(shooter.velocityControllerCommands.setTarget(ShooterTunables.SPEED_OVERRIDE_4));

        // Trigger leftStickUp = new Trigger(() -> -operatorController.getLeftY() > 0.8);
        // Trigger leftStickDown = new Trigger(() -> -operatorController.getLeftY() < -0.8);

        // leftStickUp.debounce(0.05).onTrue(shooter.nudgeRPM(200));
        // leftStickDown.debounce(0.05).onTrue(shooter.nudgeRPM(-200));
    }

    private void configureTestingBindings() {
        // this deploy command works (use for auto)
        operatorController.a().debounce(0.05).onTrue(intake.deploy());
        operatorController.b().debounce(0.05).onTrue(intake.stow());

        // use this deploy for teleOp
        operatorController.x().debounce(0.05).whileTrue(intake.startIntaking().andThen(intake.moveOut()));
        operatorController.y().debounce(0.05).whileTrue(intake.stopIntaking().andThen(intake.moveIn()));

        operatorController.rightBumper().debounce(0.05).onTrue(intake.smoothDeploy());
        operatorController.leftBumper().debounce(0.05).onTrue(intake.smoothStow());

        operatorController.povDown().debounce(0.05).onTrue(intake.suddenStow());
        operatorController.povUp().debounce(0.05).onTrue(intake.suddenDeploy());

        // operatorController.a().debounce(0.1).onTrue(simpleIntake.deploy());
        // operatorController.b().debounce(0.1).onTrue(simpleIntake.stow());

        // operatorController.x().debounce(0.1).whileTrue(simpleIntake.moveOut());
        // operatorController.y().debounce(0.1).whileTrue(simpleIntake.moveIn());

        // operatorController.rightBumper().debounce(0.1).onTrue(kicker.kick());
        // operatorController.leftBumper().debounce(0.1).onTrue(kicker.idleKicker());

        // operatorController
        //         .a()
        //         .debounce(0.05)
        //         .whileTrue(simpleIntake.startIntaking().andThen(simpleIntake.moveOut()));
        // operatorController
        //         .b()
        //         .debounce(0.05)
        //         .whileTrue(simpleIntake.stopIntaking().andThen(simpleIntake.moveIn()));

        // adjustments for testing
        // operatorController.back().debounce(0.1).onTrue(drive.headingControllerCommands.publishPIDGains());
        // operatorController.start().debounce(0.1).onTrue(drive.headingControllerCommands.updatePIDGains());

        // // set the robot to point towards 0 heading
        // operatorController
        //         .a()
        //         .debounce(0.1)
        //         .onTrue(drive.runOnce(() -> drive.setDefaultCommand(
        //                 drive.driveTopDown(() -> new ChassisSpeeds(), () -> new Rotation2d()))));

        // // set the robot to point towards 90 degrees heading
        // operatorController
        //         .b()
        //         .debounce(0.1)
        //         .onTrue(drive.runOnce(() -> drive.setDefaultCommand(
        //                 drive.driveTopDown(() -> new ChassisSpeeds(), () -> new Rotation2d(Math.PI / 2)))));

        // // coarse adjustment
        // operatorController.povUp().debounce(0.1).onTrue(shooter.nudgeRPM(250));

        // operatorController.povDown().debounce(0.1).onTrue(shooter.nudgeRPM(-250));

        // // fine adjustment
        // operatorController.povRight().debounce(0.1).onTrue(shooter.nudgeRPM(1000));

        // operatorController.povLeft().debounce(0.1).onTrue(shooter.nudgeRPM(-1000));
    }

    private ChassisSpeeds getJoystickSpeeds() {
        return new ChassisSpeeds(
                MathUtil.applyDeadband(driverController.getLeftX(), DriveBaseTunables.INPUT_DEADZONE, 1.0)
                        * DriveBaseTunables.DRIVE_SPEED,
                MathUtil.applyDeadband(driverController.getLeftY(), DriveBaseTunables.INPUT_DEADZONE, 1.0)
                        * DriveBaseTunables.DRIVE_SPEED,
                MathUtil.applyDeadband(driverController.getRightX(), DriveBaseTunables.INPUT_DEADZONE, 1.0)
                        * DriveBaseTunables.TURN_SPEED);
    }

    public Command getAutonomousCommand() {
        return Commands.sequence(enterManualMode(), autoChooser.get()).finallyDo(() -> {
            autoKickerModeEnabled = true;
            driverCanChangeShooterRPM = true;
        });
        // return drive.nudgeBack()
        //         .withTimeout(3)
        //         .andThen(DriverCommands.setAimAtTarget(
        //                 drive, shooter, onBlueAlliance, () -> new ChassisSpeeds(), calculatedHubTarget, () -> true))
        //         .andThen(Commands.waitUntil(robotReadyToShoot))
        //         .andThen(kicker.startKicker())
        //         .andThen(vision.deactivateLEDMode());
    }
}
