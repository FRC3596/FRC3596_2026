// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class RollerSub extends SubsystemBase {
    private final SparkMax Roller1 = new SparkMax(Constants.CANBus.RollerIntake1, MotorType.kBrushless);
  private SparkMaxConfig R1Config = new SparkMaxConfig();
  /** Creates a new RollerSub. */
  public RollerSub() {
     Roller1.configure(R1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }
 public void runRoller( double rollerSpeed) {
  
    Roller1.set(rollerSpeed);
   
   
 
    SmartDashboard.putNumber("Subsystem set roller speed", rollerSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
