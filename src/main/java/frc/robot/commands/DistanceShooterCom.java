// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;
import frc.robot.utils.Constants;
import frc.robot.subsystems.ShooterSub;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DistanceShooterCom extends Command {

  private final SwerveSub m_SwerveSub;
  private final ShooterSub m_ShooterSub;

  /** Creates a new DistanceShooterCom. */
  public DistanceShooterCom(ShooterSub ShooterSub, SwerveSub SwerveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_SwerveSub = SwerveSub;
    m_ShooterSub = ShooterSub;
    addRequirements(ShooterSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double poseY = m_SwerveSub.getPose().getY();
    double poseX = m_SwerveSub.getPose().getX();
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    double goalX = 0;
    double goalY = 0;
    if (alliance.isPresent()) {
      switch (alliance.get()) {
        case Blue:
          goalX = Constants.FieldConstants.blueGoalX;
          goalY = Constants.FieldConstants.blueGoalY;
          break;

        case Red:
        default:
          goalX = Constants.FieldConstants.redGoalX;
          goalY = Constants.FieldConstants.redGoalY;

      }
    }
    double robotGoalDist = Math.pow(((Math.pow(goalX - poseX, 2)))+(Math.pow(goalY-poseY,2)), 1/2);
    // TODO turn robotGoalDist to RPM using kinematics which then do PID

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
