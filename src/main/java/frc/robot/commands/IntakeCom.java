// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;
import frc.robot.utils.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCom extends Command {
  private final IntakeSub m_intakeSub;
  private final double m_speed;
  
//RyAn Wsa HerE
  /** Creates a new ManualIntakeCom. */
  public IntakeCom(IntakeSub intakeSub) {
    m_intakeSub = intakeSub;
    m_speed = Constants.Manipulator.autoIntakeSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSub);
  }
  public IntakeCom(IntakeSub intakeSub, double speed) {
    m_intakeSub = intakeSub;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSub);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSub.runIntake(0);
    m_intakeSub.motorPoseSet(Constants.Manipulator.intakeDownRotations);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSub.runIntake(m_speed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.runIntake(0);
      m_intakeSub.motorPoseSet(Constants.Manipulator.intakeUpRotations);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
