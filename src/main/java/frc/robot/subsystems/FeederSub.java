// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class FeederSub extends SubsystemBase {
  private final SparkMax feeder = new SparkMax(Constants.CANBus.feeder, MotorType.kBrushless);


  public FeederSub() {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runFeeder(double speed) {
    feeder.set(speed);
  }
}
