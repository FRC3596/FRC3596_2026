// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.FeederSub;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.Command;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeederCom extends Command {

private final FeederSub m_feederSub;
  /** Creates a new FeederCom. */
  public FeederCom(FeederSub feederSub) {
m_feederSub = feederSub; 
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    m_feederSub.runFeeder(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 m_feederSub.runFeeder(Constants.Manipulator.FeederSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_feederSub.runFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
