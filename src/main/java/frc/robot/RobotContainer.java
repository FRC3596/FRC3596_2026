// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FeederSub;
//import frc.robot.subsystems.ClimberSub;
//import frc.robot.subsystems.FeederSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.StorageSub;
import frc.robot.utils.Constants;
import frc.robot.commands.AgitatorCom;
import frc.robot.commands.FeederCom;
import frc.robot.commands.FlyStickDrive;
//import frc.robot.commands.FeederCom;
import frc.robot.commands.IntakeCom;
import frc.robot.commands.RollerCom;
import frc.robot.commands.ShooterCom;
import frc.robot.commands.ShooterIdleCom;
//import frc.robot.commands.ShooterIdleCom;
import frc.robot.commands.XboxDriveCom;
import frc.robot.subsystems.SwerveSub;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private final IntakeSub m_intakeSub = new IntakeSub();
  private final ShooterSub m_shooterSub = new ShooterSub();
  private final FeederSub m_FeederSub = new FeederSub();
  private final StorageSub m_storageSub = new StorageSub(m_shooterSub);

  private final CommandXboxController m_driverController = new CommandXboxController(
      Constants.DriverStation.xboxControllerID);
  private final CommandJoystick m_LeftJoystick = new CommandJoystick(Constants.DriverStation.leftFlightStickID);
  private final CommandJoystick m_RightJoystick = new CommandJoystick(Constants.DriverStation.rightFlightStickID);
  private final AgitatorCom m_agitatorCom = new AgitatorCom(m_storageSub, Constants.Manipulator.autoAgitatorSpeed);
  // private final AgitatorCom m_invertedAgitatorCom = new
  // AgitatorCom(m_storageSub, -Constants.Manipulator.autoAgitatorSpeed);
  private final IntakeCom m_intakeDownCom = new IntakeCom(m_intakeSub, Constants.Manipulator.intakeDownRotations);
  private final IntakeCom m_intakeUpCom = new IntakeCom(m_intakeSub, Constants.Manipulator.intakeUpRotations);
  private final ShooterCom m_shooterComFar = new ShooterCom(m_shooterSub, Constants.Manipulator.LongShooterSpeed);
  private final ShooterCom m_shooterComClose = new ShooterCom(m_shooterSub, Constants.Manipulator.MediumShooterSpeed);
  private final ShooterCom m_shooterSlow = new ShooterCom(m_shooterSub, Constants.Manipulator.ShortShooterSpeed);
  private final ShooterCom m_shooterOff = new ShooterCom(m_shooterSub, 0);
  private final RollerCom m_RollerInCom = new RollerCom(m_intakeSub, Constants.Manipulator.autoIntakeSpeed);
  private final RollerCom m_RollerOutCom = new RollerCom(m_intakeSub, -Constants.Manipulator.autoIntakeSpeed);
  private final FeederCom m_FeederCom = new FeederCom(m_FeederSub);
  // private final ShooterIdleCom Idle = new ShooterIdleCom(m_shooterSub);;
  // //private final FeederCom m_FeederCom = new FeederCom(m_FeederSub);
  // private final ClimberSub m_ClimberSub = new ClimberSub();

  private final SwerveSub swerve = new SwerveSub();
  CommandXboxController xboxController = new CommandXboxController(Constants.DriverStation.xboxControllerID);
  XboxDriveCom teleopXBoxDriveCommand = new XboxDriveCom(swerve, xboxController);
  FlyStickDrive teleopFlyStickDriveCommand = new FlyStickDrive(swerve, m_LeftJoystick, m_RightJoystick);

  public RobotContainer() {

    configureBindings();
    swerve.setDefaultCommand(teleopFlyStickDriveCommand);

    autoChooser = AutoBuilder.buildAutoChooser();
    NamedCommands.registerCommand("FarFire", m_shooterComFar);
    NamedCommands.registerCommand("CloseFire", m_shooterComFar);
    NamedCommands.registerCommand("Idle", m_shooterSlow);
    NamedCommands.registerCommand("Agitate", m_agitatorCom);
    NamedCommands.registerCommand("Off", m_shooterOff);
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {
    m_driverController.b().whileTrue(m_shooterComFar);
    m_driverController.y().whileTrue(m_shooterComClose);
    // m_driverController.x().whileTrue(m_shooterSlow);
    m_driverController.x().whileTrue(m_shooterOff);
    // m_driverController.a().whileTrue(m_agitatorCom);
    // m_driverController.a().whileTrue(m_FeederCom);
    // m_driverController.rightBumper().whileTrue(m_intakeDownCom);
    // m_driverController.leftBumper().whileTrue(m_intakeUpCom);
    // m_driverController.axisGreaterThan(3, 0.5).whileTrue(m_RollerInCom);
    // m_driverController.axisGreaterThan(2, 0.5).whileTrue(m_RollerOutCom);
    // m_driverController.button(9).whileTrue(m_invertedAgitatorCom);
    // m_driverController.rightTrigger().whileTrue(m_FeederCom); */
    m_RightJoystick.button(3).onTrue(new InstantCommand(swerve::resetPose));
    // m_driverController.povUp()
    // .onTrue(new InstantCommand(() ->
    // m_ClimberSub.motorPoseSet(Constants.Manipulator.climberRotationsUp)));
    // m_driverController.povDown()
    // .onTrue(new InstantCommand(() ->
    // m_ClimberSub.motorPoseSet(Constants.Manipulator.climberRotationsDown)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

    // swerve.setDefaultCommand(teleopDriveCommand);
    // m_shooterSub.setDefaultCommand(Idle);
  }

}
