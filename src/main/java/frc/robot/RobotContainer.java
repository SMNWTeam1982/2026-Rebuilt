// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.HubCommands;
import frc.robot.Constants.Measured.FieldMeasurements;
import frc.robot.Constants.Tunables.ShooterTunables;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import java.util.Optional;

public class RobotContainer {

    private final DriveSubsystem drive = new DriveSubsystem(() -> Optional.empty());

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        boolean onBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;

        Translation2d hubCenter;

        if (onBlueAlliance) {
            hubCenter = FieldMeasurements.BLUE_HUB_CENTER;
        } else {
            hubCenter = FieldMeasurements.RED_HUB_CENTER;
        }

        Trigger confidentShot = new Trigger(() -> {
            Pose2d robotPose = drive.getRobotPose();
            Translation2d newHubPosition = HubCommands.calculateScoringTarget(
                    robotPose.getTranslation(), hubCenter, drive.getFieldRelativeVelocity());

            double futureDistanceFromHub = newHubPosition.getDistance(robotPose.getTranslation());
            double currentDistanecFromHub = robotPose.getTranslation().getDistance(hubCenter);

            // // if the distance from the hub changes by a negligable amount in the future then the shot confidence is
            // high
            return Math.abs(futureDistanceFromHub - currentDistanecFromHub)
                    <= ShooterTunables.SHOOTING_POSITION_TOLERANCE;
        });
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
