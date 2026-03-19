// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class IntakeSub extends SubsystemBase {

  private final SparkMax Intake1 = new SparkMax(Constants.CANBus.pivotIntake1, MotorType.kBrushless);
  private final SparkMax Intake2 = new SparkMax(Constants.CANBus.pivotIntake2, MotorType.kBrushless);
  private SparkMaxConfig p2Config = new SparkMaxConfig();
  private SparkMaxConfig p1Config = new SparkMaxConfig();
  private final RelativeEncoder p1encoder = Intake1.getEncoder();
  private double pastCurrentOutput;
  private double pastPosition;
  private boolean trip;
  private double tripOutput;

  public IntakeSub() {

    p1Config.smartCurrentLimit(20);
    Intake1.configure(p1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    p2Config.follow(Intake1, true);
    Intake2.configure(p2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    if (pastPosition != p1encoder.getPosition()) {
      if (Math.abs((Intake1.getOutputCurrent() - pastCurrentOutput)/(p1encoder.getPosition() - pastPosition)) > Constants.Manipulator.currentDerivLim) {
        tripOutput = Intake1.get();
        trip = true;
      }

    }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot encoder", p1encoder.getPosition());

    pastCurrentOutput = Intake1.getOutputCurrent();
    pastPosition = p1encoder.getPosition();

  }

  public void runIntake(double speed) {
    if(!(((speed/tripOutput) > 0) && trip)) {  
      Intake1.set(speed);
      trip = false; 
    }
  }

  public void motorPoseSet(double PoseRotations) {

  }
}
