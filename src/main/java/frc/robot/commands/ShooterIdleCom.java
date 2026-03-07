// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.utils.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterIdleCom extends Command {

  private final ShooterSub m_shooterSub;
  private final double m_idleSpeed;

  /** Creates a new ShooterIdleCom. */
  public ShooterIdleCom(ShooterSub shooterSub) {
    m_shooterSub = shooterSub;
    m_idleSpeed = Constants.Manipulator.idleSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);// Use addRequirements() here to declare subsystem dependencies.
  }
   public ShooterIdleCom(ShooterSub shooterSub, double idleSpeed) {
    m_shooterSub = shooterSub;
    m_idleSpeed = idleSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);// Use addRequirements() here to declare subsystem dependencies.
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSub.customIdleSpeed(m_idleSpeed);
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
