// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class StorageSub extends SubsystemBase {
  private final SparkMax LAgitator = new SparkMax(Constants.CANBus.LAgitator, MotorType.kBrushless);
  private final SparkMax RAgitator = new SparkMax(Constants.CANBus.RAgitator, MotorType.kBrushless);
  private SparkBaseConfig LAConfig;
  /** Creates a new StorageSub. */
  public StorageSub() {
    LAConfig.follow(RAgitator);
    LAgitator.configure(LAConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void runAgitator(double speed){
    LAgitator.set(speed);
  }
}
