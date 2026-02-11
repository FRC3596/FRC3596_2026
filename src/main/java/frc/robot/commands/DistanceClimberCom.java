// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DistanceClimberCom extends Command {
  /** Creates a new DistanceClimberCom. */
   private final SwerveSub m_SwerveSub;
  private final ShooterSub m_ClimberSub;

  public DistanceClimberCom(ShooterSub ClimberSub, SwerveSub SwerveSub) {
     m_SwerveSub = SwerveSub;
    m_ClimberSub = ClimberSub;
    addRequirements(ClimberSub);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double poseY = m_SwerveSub.getPose().getY();
    double poseX = m_SwerveSub.getPose().getX();
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    //climberX and climberY represent climber's x and y coordinate positions relative to the field
    double climberX = 0;
    double climberY = 0;
    if (alliance.isPresent()) {
      switch (alliance.get()) {
        case Blue:
          climberX = Constants.FieldConstants.blueClimberX;
          climberY = Constants.FieldConstants.blueClimberY;
          break;

        case Red:
        default:
          climberX = Constants.FieldConstants.redClimberX;
          climberY = Constants.FieldConstants.redClimberY;

      }
    }
    double robotToClimberDist = Math.pow(((Math.pow((climberX - poseX), 2)))+(Math.pow((climberY-poseY),2)), 1/2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
