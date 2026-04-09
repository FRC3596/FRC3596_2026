// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RollerCom extends Command {
  private final IntakeSub m_IntakeSub;
  private final double m_Speed;
  private final double m_targetPose;
  /** Creates a new RollerCom. */
  public RollerCom(IntakeSub intakeSub, double speed, double desiredPose) {
    m_IntakeSub = intakeSub;
    m_Speed = speed;
    m_targetPose = desiredPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSub.runIntake(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSub.runIntake(m_Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSub.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
