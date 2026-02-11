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
  private final SparkMax Roller2 = new SparkMax(Constants.CANBus.Intake2, MotorType.kBrushless);
  private final SparkMax pivotIntake = new SparkMax(Constants.CANBus.pivotIntake, MotorType.kBrushless);
  private SparkMaxConfig I1Config = new SparkMaxConfig();
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private ClosedLoopConfig PIDConfig = new ClosedLoopConfig();
  private final SparkClosedLoopController PID = pivotIntake.getClosedLoopController();

  /** Creates a new IntakeSub. */
  public IntakeSub() {

    pivotConfig.apply(PIDConfig);
    pivotIntake.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    PIDConfig.pid(Constants.Manipulator.pivotProportion, Constants.Manipulator.pivotIntegral,
        Constants.Manipulator.pivotDerivative);

    I1Config.follow(Roller2);
    I1Config.inverted(true);
    Roller1.configure(I1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake(double speed) {
    Roller1.set(speed);
  }

  public void motorVeloSet(double PoseRotations) {

    PID.setSetpoint(PoseRotations, SparkBase.ControlType.kPosition);
  }

}