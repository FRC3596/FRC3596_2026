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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class ShooterSub extends SubsystemBase {
  private final SparkMax motor = new SparkMax(Constants.CANBus.shooter, MotorType.kBrushless);
  private SparkMaxConfig shooterConfig = new SparkMaxConfig();
  private ClosedLoopConfig PIDConfig = new ClosedLoopConfig();
  private final SparkClosedLoopController PID = motor.getClosedLoopController();
  private final RelativeEncoder encoder = motor.getEncoder();

  /** Creates a new ShooterSub. */
  public ShooterSub() {

    PIDConfig.pid(Constants.Manipulator.shooterProportion, Constants.Manipulator.shooterIntegral,
        Constants.Manipulator.shooterDerivative).outputRange(0, 1);
    shooterConfig.idleMode(IdleMode.kCoast);
    shooterConfig.inverted(true);
    shooterConfig.apply(PIDConfig);
    
    motor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    public void defaultIdleSpeed() {
  customIdleSpeed(Constants.Manipulator.idleSpeed);
  }


  public void customIdleSpeed(double customIdleSpeed) {
    if ((encoder.getVelocity() > customIdleSpeed)) {
      motor.set(0);
    } else {
      PID.setSetpoint(customIdleSpeed, SparkBase.ControlType.kVelocity);

    }
  }

  public void motorVeloSet(double speedRPM) {

    PID.setSetpoint(speedRPM, SparkBase.ControlType.kVelocity);

  }

}
