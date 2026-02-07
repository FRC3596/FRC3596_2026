// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.LimeLightDriveAlignment;
import frc.robot.commands.XboxDrive;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.utils.Constants;


public class RobotContainer {

  private final SwerveSub swerve = new SwerveSub();
  private final CommandXboxController xboxController = new CommandXboxController(Constants.DriverStation.xboxControllerID);
  private final XboxDrive teleopDriveCommand = new XboxDrive(swerve, xboxController);
  private final LimeLightSub limelight = new LimeLightSub(swerve);

  private final LimeLightDriveAlignment limelightDriveAlignmentCommand = new LimeLightDriveAlignment(limelight, swerve);
  private final LimeLightDriveAlignment limelightDriveAlignmentCommandTest = new LimeLightDriveAlignment(limelight, swerve, true);
  public RobotContainer() {
    SmartDashboard.putNumber("Test Horizontal Setpoint", 0);
    SmartDashboard.putNumber("Test Vertical Setpoint", 0);
    SmartDashboard.putNumber("Test Rotation Setpoint", 0);
    
    configureBindings();
    swerve.setDefaultCommand(teleopDriveCommand);
  }

  private void configureBindings() {
    xboxController.a().whileTrue(limelightDriveAlignmentCommand);
    xboxController.b().whileTrue(limelightDriveAlignmentCommandTest);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
