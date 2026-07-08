// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class StorageSub extends SubsystemBase {
  // private final SparkMax agitator = new SparkMax(Constants.CANBus.agitator, MotorType.kBrushless);
 
  private SparkMaxConfig agitatorConfig = new SparkMaxConfig();
  /** Creates a new StorageSub. */
  private final ShooterSub m_ShooterSub;
  public StorageSub(ShooterSub shooterSub) {
    m_ShooterSub = shooterSub;

    // agitator.configure(agitatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void runAgitator(double speed){
    // if (m_ShooterSub.ShooterVelocity() >= 2000)
    // { 
    //   agitator.set(speed);
    // }
    // else {
    //   agitator.set(0);
    // }
  }
} 

