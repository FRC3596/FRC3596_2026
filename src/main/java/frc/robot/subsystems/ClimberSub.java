// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ClimberSub extends SubsystemBase {
  private final SparkMax climberL = new SparkMax(Constants.CANBus.climb1, MotorType.kBrushless);
  private final SparkMax climberR = new SparkMax(Constants.CANBus.climb2, MotorType.kBrushless);
  private SparkMaxConfig climberLConfig = new SparkMaxConfig();
  private SparkMaxConfig climberRConfig = new SparkMaxConfig();
  private ClosedLoopConfig PIDConfig = new ClosedLoopConfig();
  private final SparkClosedLoopController PIDL = climberL.getClosedLoopController();
  private final SparkClosedLoopController PIDR = climberR.getClosedLoopController();

  /** Creates a new ClimberSub. */
  public ClimberSub() {
    PIDConfig.pid(Constants.Manipulator.climberLProportion, Constants.Manipulator.climberLIntegral,
      Constants.Manipulator.climberLDerivative).outputRange(-1, 1);
    climberLConfig.apply(PIDConfig);
    climberL.configure(climberLConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
    PIDConfig.pid(Constants.Manipulator.climberRProportion, Constants.Manipulator.climberRIntegral,
      Constants.Manipulator.climberRDerivative).outputRange(-1, 1);
    climberRConfig.apply(PIDConfig);
    climberR.configure(climberRConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorPoseSet(double PoseRotations) {

    PIDL.setSetpoint(PoseRotations, SparkBase.ControlType.kPosition);
    PIDR.setSetpoint(PoseRotations, SparkBase.ControlType.kPosition);
  }
}
