// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command.AgitatorCom;
import frc.robot.command.IntakeCom;
import frc.robot.command.ShooterCom;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.StorageSub;
import frc.robot.utils.Constants;

public class RobotContainer {
  private final StorageSub m_storageSub = new StorageSub();
  private final IntakeSub m_intakeSub = new IntakeSub();
  private final ShooterSub m_shooterSub = new ShooterSub();
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.DriverStation.xboxControllerID);
  private final AgitatorCom m_agitatorCom = new AgitatorCom(m_storageSub, Constants.Manipulator.teleopAgitatorSpeed);
  private final IntakeCom m_intakeCom = new IntakeCom(m_intakeSub, Constants.Manipulator.teleopIntakeSpeed);
  private final ShooterCom m_shooterCom = new ShooterCom(m_shooterSub, Constants.Manipulator.teleopShooterSpeed);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().whileTrue(m_intakeCom);
    m_driverController.b().whileTrue(m_shooterCom);
    m_driverController.y().whileTrue(m_agitatorCom);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");

  }
}
