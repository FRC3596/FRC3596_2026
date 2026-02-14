// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class IntakeSub extends SubsystemBase {
  private final SparkMax Roller1 = new SparkMax(Constants.CANBus.Intake1, MotorType.kBrushless);
  private final SparkMax pivotIntake1 = new SparkMax(Constants.CANBus.pivotIntake1, MotorType.kBrushless);
  private final SparkMax pivotIntake2 = new SparkMax(Constants.CANBus.pivotIntake2, MotorType.kBrushless);
  private SparkMaxConfig p2Config = new SparkMaxConfig();
  private SparkMaxConfig p1Config = new SparkMaxConfig();
  private ClosedLoopConfig PIDConfig = new ClosedLoopConfig();
  private final SparkClosedLoopController pivotPID = pivotIntake1.getClosedLoopController();

  /** Creates a new IntakeSub. */
  public IntakeSub() {

    PIDConfig.pid(Constants.Manipulator.pivotProportion, Constants.Manipulator.pivotIntegral,
        Constants.Manipulator.pivotDerivative);
    p1Config.apply(PIDConfig);
    p2Config.follow(pivotIntake1, true);

    pivotIntake1.configure(p1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    pivotIntake2.configure(p2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {
    Roller1.set(speed);
  }

  public void motorPoseSet(double PoseRotations) {

    pivotPID.setSetpoint(PoseRotations, SparkBase.ControlType.kPosition);
  }

}