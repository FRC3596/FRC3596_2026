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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ShooterSub extends SubsystemBase {
  private final SparkMax shooter = new SparkMax(Constants.CANBus.shooter, MotorType.kBrushless);
  private SparkMaxConfig shooterConfig = new SparkMaxConfig();
  private ClosedLoopConfig PIDConfig = new ClosedLoopConfig();
  private final SparkClosedLoopController PID = shooter.getClosedLoopController();

  /** Creates a new ShooterSub. */
  public ShooterSub() {
    
    PIDConfig.pid(Constants.Manipulator.shooterProportion, Constants.Manipulator.shooterIntegral,
        Constants.Manipulator.shooterDerivative);
    shooterConfig.idleMode(IdleMode.kCoast);
    shooterConfig.apply(PIDConfig);
    shooter.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void motorVeloSet(double speedRPM) {

    PID.setSetpoint(speedRPM, SparkBase.ControlType.kVelocity);
  }

}
