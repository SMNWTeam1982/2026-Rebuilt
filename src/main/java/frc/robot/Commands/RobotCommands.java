package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Tunables;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Kicker.KickerSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShotCalculation;
import java.util.function.BooleanSupplier;

public class RobotCommands {
    public static Command setShooterRPMSupplierForAllianceHub(
            DriveSubsystem drive, ShooterSubsystem shooter, BooleanSupplier onBlueAlliance) {
        return shooter.setRPMSupplier(() -> ShotCalculation.calculateAllianceHubRPM(
                drive.getRobotPose().getTranslation(), onBlueAlliance.getAsBoolean()));
    }

    public static Command enterDefenseMode(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
        return Commands.parallel(shooter.turnOff(), kicker.turnOff(), intake.turnOff());
    }

    public static Command exitDefenseMode(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
        return Commands.parallel(shooter.turnOn(), kicker.turnOn(), intake.turnOn());
    }

    public static Command tryUnjam(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
        return Commands.sequence(
                        Commands.parallel(
                                shooter.setReverse(), // set shooter to reverse RPM
                                kicker.setReverse(),
                                intake.setReverse()),
                        shooter.runPIDs()
                                .withTimeout(Tunables.UNJAM_ATTEMPT_TIME), // give subsystems some time to try and unjam
                        Commands.parallel( // if the subsystems are still jammed after then disable their jammed
                                // components
                                shooter.turnOffJammedChannels(), intake.turnOffJammedIntake()),
                        Commands.parallel(shooter.setIdle(), kicker.setIdle(), intake.stopIntaking()))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
