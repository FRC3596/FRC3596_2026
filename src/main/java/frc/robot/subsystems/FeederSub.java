// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class FeederSub extends SubsystemBase {
  private final SparkMax feeder = new SparkMax(Constants.CANBus.feeder, MotorType.kBrushless);
  private SparkMaxConfig feederConfig = new SparkMaxConfig();
  private ClosedLoopConfig PIDConfig = new ClosedLoopConfig();
  private final SparkClosedLoopController PID = feeder.getClosedLoopController();

  public FeederSub() {
    feederConfig.apply(PIDConfig);
    feeder.configure(feederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    PIDConfig.pid(Constants.Manipulator.feederProportion, Constants.Manipulator.feederIntegral,
        Constants.Manipulator.feederDerivative);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runFeeder(double speed) {
    feeder.set(speed);
  }

  public void motorVeloSet(double speedRPM) {

    PID.setSetpoint(speedRPM, SparkBase.ControlType.kVelocity);
  }
}
