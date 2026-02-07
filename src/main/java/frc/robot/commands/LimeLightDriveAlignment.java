// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.utils.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimeLightDriveAlignment extends Command {
  private final LimeLightSub m_limeLightSub;
  private final SwerveSub m_swerveSub;
  private final double m_HorizontalSetPoint;
  private final double m_VerticalSetPoint;
  private final double m_RotationSetPoint;
  /** Creates a new LimeLightDriveAlignment. */
  public LimeLightDriveAlignment(LimeLightSub limeLightSub, SwerveSub swerveSub) {
    m_limeLightSub = limeLightSub;
    m_swerveSub = swerveSub;
    m_HorizontalSetPoint = Constants.LimeLight.VisionPID.VisionHorizontalSetpoints[m_limeLightSub.getTargetID()];
    m_VerticalSetPoint = Constants.LimeLight.VisionPID.VisionVerticalSetpoints[m_limeLightSub.getTargetID()];
    m_RotationSetPoint = Constants.LimeLight.VisionPID.VisionRotationSetpoints[m_limeLightSub.getTargetID()];
    addRequirements(m_limeLightSub, m_swerveSub);
  }
  public LimeLightDriveAlignment(LimeLightSub limeLightSub, SwerveSub swerveSub, boolean TestMode) {
    m_limeLightSub = limeLightSub;
    m_swerveSub = swerveSub;
    m_HorizontalSetPoint = SmartDashboard.getNumber("Test Horizontal Setpoint", 0);
    m_VerticalSetPoint = SmartDashboard.getNumber("Test Vertical Setpoint", 0);
    m_RotationSetPoint = SmartDashboard.getNumber("Test Rotation Setpoint", 0);
    addRequirements(m_limeLightSub, m_swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveSub.control(0, 0, 0);
    m_limeLightSub.addVisionHorizontalSetpoint(m_HorizontalSetPoint);
    m_limeLightSub.addVisionVerticalSetpoint(m_VerticalSetPoint);
    m_limeLightSub.addVisionRotationSetpoint(m_RotationSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limeLightSub.hasTarget()) {
    m_swerveSub.control(m_limeLightSub.CalculateVisionHorizontalPID(m_HorizontalSetPoint), 
                          m_limeLightSub.CalculateVisionVerticalPID(m_VerticalSetPoint), 
                            m_limeLightSub.CalculateVisionRotationPID(m_RotationSetPoint));
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!m_limeLightSub.hasTarget()) {
      SmartDashboard.putString("LimeLight Status", "No Target");
    }
    m_swerveSub.control(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limeLightSub.atSetPoint() || !m_limeLightSub.hasTarget();
  }
}
