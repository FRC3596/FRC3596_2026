// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.XboxDrive;
import frc.robot.subsystems.SwerveSub;
import frc.robot.utils.Constants;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

 SwerveSub swerve = new SwerveSub();
 CommandXboxController xboxController = new CommandXboxController(Constants.DriverStation.xboxControllerID);
XboxDrive teleopDriveCommand = new XboxDrive(swerve, xboxController);

  public RobotContainer() {
    configureBindings();
    swerve.setDefaultCommand(teleopDriveCommand);

    autoChooser = AutoBuilder.buildAutoChooser();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
