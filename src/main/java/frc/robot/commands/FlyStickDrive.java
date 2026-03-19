
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSub;
import frc.robot.utils.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlyStickDrive extends Command {

  CommandXboxController xboxController;
  private final SwerveSub m_swerve;
  private final CommandJoystick m_leftJoystick;
  private final CommandJoystick m_rightJoystick;

  /** Creates a new XboxDrive. */
  public FlyStickDrive(SwerveSub swerve, CommandJoystick leftJoystick, CommandJoystick rightJoystick) {
    m_swerve = swerve;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;
addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  // XBox controller controls swerve modules
  @Override
  public void execute() {
    m_swerve.control(Constants.DriverStation.DriveSpeedMultiplier * MathUtil.applyDeadband(m_leftJoystick.getX(), 0.05), 
                     Constants.DriverStation.DriveSpeedMultiplier * MathUtil.applyDeadband(m_leftJoystick.getY(),0.05),
                     12* MathUtil.applyDeadband(m_rightJoystick.getX(), 0.05));
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
