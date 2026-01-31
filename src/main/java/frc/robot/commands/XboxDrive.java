// Copyright (c) bob and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSub;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class XboxDrive extends Command {

  CommandXboxController xboxController;
  SwerveSub swerve;

  /** Creates a new XboxDrive. */
  public XboxDrive(SwerveSub swerve, CommandXboxController xboxController) {
    this.swerve = swerve;
    this.xboxController = xboxController;
addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  // XBox controller controls swerve modules
  @Override
  public void execute() {
    swerve.control(xboxController.getLeftX(),-xboxController.getLeftY(),xboxController.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
