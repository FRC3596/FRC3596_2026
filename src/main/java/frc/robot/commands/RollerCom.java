// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.RollerSub;
import frc.robot.utils.Constants;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RollerCom extends Command {
  private final IntakeSub m_IntakeSub;
   private final RollerSub m_RollerSub;
  private final double m_Speed;

  /** Creates a new RollerCom. */
  public RollerCom(IntakeSub intakeSub, RollerSub rollerSub, double speed)  {
    m_RollerSub = rollerSub;
    m_IntakeSub = intakeSub;
    m_Speed = speed;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSub);
    addRequirements(m_RollerSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_RollerSub.runRoller(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_IntakeSub.ReturnEncoder())< Constants.Manipulator.minPoseForRoller){
         m_RollerSub.runRoller(0);
    }
    else{
 m_RollerSub.runRoller(m_Speed);
    }
    SmartDashboard.putNumber("set roller speed", m_Speed);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_RollerSub.runRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
