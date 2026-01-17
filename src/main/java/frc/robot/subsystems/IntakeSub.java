// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class IntakeSub extends SubsystemBase {
  private final SparkMax LIntake = new SparkMax(Constants.CANBus.LIntake, MotorType.kBrushless);
  private final SparkMax RIntake = new SparkMax(Constants.CANBus.RIntake, MotorType.kBrushless);
  private SparkBaseConfig LIConfig;
  /** Creates a new IntakeSub. */
  public IntakeSub() {
    LIConfig.follow(RIntake);
    LIntake.configure(LIConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runIntake(double speed){
    LIntake.set(speed);
  }
}
