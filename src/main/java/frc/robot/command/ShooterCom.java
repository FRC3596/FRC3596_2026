// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.utils.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCom extends Command {
  private final ShooterSub m_shooterSub;
  private final double m_speed;

  /** Creates a new ShooterCom. */
  public ShooterCom(ShooterSub shooterSub) {
    m_shooterSub = shooterSub;
    m_speed = Constants.Manipulator.autoShooterSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSub);
  }
  public ShooterCom(ShooterSub shooterSub, double speed) {
    m_shooterSub = shooterSub;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSub.runShooter(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSub.runShooter(m_speed);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSub.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
