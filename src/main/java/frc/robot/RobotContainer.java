// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.StorageSub;
import frc.robot.utils.Constants;
import frc.robot.commands.AgitatorCom;
import frc.robot.commands.FeederCom;
import frc.robot.commands.IntakeCom;
import frc.robot.commands.ShooterCom;
import frc.robot.commands.ShooterIdleCom;
import frc.robot.commands.XboxDriveCom;
import frc.robot.subsystems.SwerveSub;

public class RobotContainer {
  private final StorageSub m_storageSub = new StorageSub();
  private final IntakeSub m_intakeSub = new IntakeSub();
  private final ShooterSub m_shooterSub = new ShooterSub();
  private final FeederSub m_FeederSub = new FeederSub();
  private final CommandXboxController m_driverController = new CommandXboxController(
      Constants.DriverStation.xboxControllerID);
  private final AgitatorCom m_agitatorCom = new AgitatorCom(m_storageSub, Constants.Manipulator.teleopAgitatorSpeed);
  private final IntakeCom m_intakeCom = new IntakeCom(m_intakeSub, Constants.Manipulator.intakeRollerSpeed);
  private final ShooterCom m_shooterCom = new ShooterCom(m_shooterSub, Constants.Manipulator.teleopShooterSpeed);
  private final FeederCom m_FeederCom = new FeederCom(m_FeederSub);
  private final ClimberSub m_ClimberSub = new ClimberSub();

  SwerveSub swerve = new SwerveSub();
  CommandXboxController xboxController = new CommandXboxController(Constants.DriverStation.xboxControllerID);
  XboxDriveCom teleopDriveCommand = new XboxDriveCom(swerve, xboxController);
  // ShooterIdleCom Idle = new ShooterIdleCom(m_shooterSub);

  public RobotContainer() {
    configureBindings();

    // swerve.setDefaultCommand(teleopDriveCommand);
    // m_shooterSub.setDefaultCommand(Idle);
  }

  private void configureBindings() {
    m_driverController.a().whileTrue(m_intakeCom);
    m_driverController.b().whileTrue(m_shooterCom);
    // m_driverController.y().whileTrue(m_agitatorCom);
    // m_driverController.rightTrigger().whileTrue(m_FeederCom);
    // m_driverController.povUp()
    //     .onTrue(new InstantCommand(() -> m_ClimberSub.motorPoseSet(Constants.Manipulator.climberRotationsUp)));
    // m_driverController.povDown()
    //     .onTrue(new InstantCommand(() -> m_ClimberSub.motorPoseSet(Constants.Manipulator.climberRotationsDown)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");

  }
}
