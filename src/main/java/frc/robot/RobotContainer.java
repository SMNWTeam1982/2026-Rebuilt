// Copyright (c) FIRST and other WPILib contributors. 
 
package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


 
public class RobotContainer {
 private final SendableChooser<Command> autoChooser;
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser("test");
    SmartDashboard.putData("auto mode", autoChooser);
    NamedCommands.registerCommand("score ", Commands.runOnce(()->{System.out.println("we scored!");}));


    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }







}
