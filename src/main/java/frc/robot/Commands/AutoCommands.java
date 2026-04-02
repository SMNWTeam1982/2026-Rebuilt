package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Kicker.KickerSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShotCalculation;
import java.util.function.BooleanSupplier;

public class AutoCommands {
    public static Command setShooterAllianceHubRPMSupplier(
            DriveSubsystem drive, ShooterSubsystem shooter, BooleanSupplier onBlueAlliance) {
        return shooter.setRPMSupplier(() -> ShotCalculation.calculateAllianceHubRPM(
                drive.getRobotPose().getTranslation(), onBlueAlliance.getAsBoolean()));
    }

    /** stops the drive,
     * and spins up the shooter flywheels with the RPM calculation function,
     * <p>then waits until the flywheels are up to speed before starting the kicker
     * <p>it will kick for a configurable amount of time
     * <p>while it is waiting and kickin it will rotate to face the nearest hub
     * <p>when done it sets the shooter to it's idle speed, and stops the drive and kicker */
    public static Command shootIntoHub(
            DriveSubsystem drive,
            ShooterSubsystem shooter,
            KickerSubsystem kicker,
            Time shootingTime,
            BooleanSupplier onBlueAlliance) {
        return Commands.sequence(
                drive.stop(),
                setShooterAllianceHubRPMSupplier(drive, shooter, onBlueAlliance)
                        .asProxy(), // run the shooter commands as proxies so that the PID can run in background
                Commands.deadline(
                        Commands.sequence(
                                Commands.waitSeconds(0.2),
                                Commands.waitUntil(shooter.readyToShoot),
                                kicker.kick().withTimeout(shootingTime)),
                        drive.driveAndPointAtTarget(
                                () -> new ChassisSpeeds(),
                                () -> ShotCalculation.getAllianceHubPosition(onBlueAlliance.getAsBoolean()))),
                kicker.setIdle(),
                shooter.setIdle().asProxy(),
                drive.stop());
    }

    /** stops the drive, then deploys the intake, ends when the intake deploy command is done */
    public static Command deployIntake(DriveSubsystem drive, IntakeSubsystem intake) {
        return Commands.sequence(drive.stop(), intake.deploy());
    }
}
