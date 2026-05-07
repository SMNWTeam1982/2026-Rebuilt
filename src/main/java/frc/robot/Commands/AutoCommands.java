package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Kicker.KickerSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShotCalculation;

public class AutoCommands {
    public static Command depot(
            DriveSubsystem drive, ShooterSubsystem shooter, KickerSubsystem kicker, boolean onBlueAlliance) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Left bump to top"))
                    .andThen(drive.stop())
                    .andThen(drive.driveAndPointAtTarget(
                                    () -> new ChassisSpeeds(),
                                    () -> ShotCalculation.getNearestHubPosition(
                                            drive.getRobotPose().getTranslation()))
                            .withTimeout(2))
                    .andThen(shooter.setTarget(
                            () -> drive.getRobotPose().getTranslation(),
                            () -> ShotCalculation.getNearestHubPosition(
                                    drive.getRobotPose().getTranslation())));
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }
}
