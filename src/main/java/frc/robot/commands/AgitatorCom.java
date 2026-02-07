// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StorageSub;
import frc.robot.utils.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitatorCom extends Command {
  private final StorageSub m_storageSub;
  private final double m_speed;
  /** Creates a new AgitatorCom. */
  public AgitatorCom(StorageSub storageSub) {
    m_storageSub = storageSub;
    m_speed = Constants.Manipulator.autoAgitatorSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_storageSub);
  }
  public AgitatorCom(StorageSub storageSub, double speed) {
    m_storageSub = storageSub;
    m_speed = speed;
    addRequirements(m_storageSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_storageSub.runAgitator(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_storageSub.runAgitator(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_storageSub.runAgitator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
