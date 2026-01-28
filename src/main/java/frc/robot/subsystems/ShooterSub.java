// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ShooterSub extends SubsystemBase {
  private final SparkMax LShooter = new SparkMax(Constants.CANBus.LShooter, MotorType.kBrushless);
  private final SparkMax RShooter = new SparkMax(Constants.CANBus.RShooter, MotorType.kBrushless);
  private SparkBaseConfig LSConfig;
  private final SparkClosedLoopController PIDLS = LShooter.getClosedLoopController();
  private final SparkClosedLoopController PIDRS = RShooter.getClosedLoopController();

  /** Creates a new ShooterSub. */
  public ShooterSub() {
    LSConfig.follow(RShooter);
    LShooter.configure(LSConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(double speed) {
    LShooter.set(speed);
  }
  public void motorVeloSet(double speedRPM){

    PIDLS.setSetpoint(speedRPM, SparkBase.ControlType.kVelocity)
  }

}
