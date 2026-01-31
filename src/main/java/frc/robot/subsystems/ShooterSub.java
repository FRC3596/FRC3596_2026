// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ShooterSub extends SubsystemBase {
  private final SparkMax shooter = new SparkMax(Constants.CANBus.LShooter, MotorType.kBrushless);
  private final SparkMax feeder = new SparkMax(Constants.CANBus.RShooter, MotorType.kBrushless);
  private SparkBaseConfig shooterConfig;
  private final SparkClosedLoopController PIDShooter = shooter.getClosedLoopController();
  private final SparkClosedLoopController PIDFeeder = feeder.getClosedLoopController();

  /** Creates a new ShooterSub. */
  public ShooterSub() {
    ClosedLoopConfig CLC = new ClosedLoopConfig();
    CLC.p(Constants.Manipulator.shooterProportion);
    CLC.i(Constants.Manipulator.shooterIntegral);
    CLC.d(Constants.Manipulator.shooterDerivative);
shooterConfig.apply(CLC);
    shooter.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(double speed) {
    shooter.set(speed);
  }

  public void motorVeloSet(double speedRPM) {

    PIDShooter.setSetpoint(speedRPM, SparkBase.ControlType.kVelocity);
  }

}
