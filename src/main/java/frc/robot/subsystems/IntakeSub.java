// Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.utils.Constants;

// public class IntakeSub extends SubsystemBase {
//   private final SparkMax Roller1 = new SparkMax(Constants.CANBus.Intake1, MotorType.kBrushless);
//   private final SparkMax pivotIntake1 = new SparkMax(Constants.CANBus.pivotIntake1, MotorType.kBrushless);
//   private final SparkMax pivotIntake2 = new SparkMax(Constants.CANBus.pivotIntake2, MotorType.kBrushless);
//   private SparkMaxConfig p2Config = new SparkMaxConfig();
//   private SparkMaxConfig p1Config = new SparkMaxConfig();
//   private ClosedLoopConfig PIDConfig = new ClosedLoopConfig();
//   private final RelativeEncoder p1encoder = pivotIntake1.getEncoder();
//   private final SparkClosedLoopController pivotPID = pivotIntake1.getClosedLoopController();
//   private double kCosValue = 2;

//   /** Creates a new IntakeSub. */
//   public IntakeSub() {
//     //PIDConfig.feedForward.kCosRatio(0);
//     kCosValue = SmartDashboard.getNumber("kCos Value", 0);
//     PIDConfig.pid(Constants.Manipulator.pivotProportion, Constants.Manipulator.pivotIntegral,
//         Constants.Manipulator.pivotDerivative).outputRange(-1, 1);
//     p1Config.apply(PIDConfig);
//     p1Config.smartCurrentLimit(10);
//     pivotIntake1.configure(p1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

//     p2Config.follow(pivotIntake1, true);
//     p2Config.smartCurrentLimit(10);
//   //  p2Config.apply(PIDConfig);
//     pivotIntake2.configure(p2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   @Override
//   public void periodic() {
//     kCosValue = SmartDashboard.getNumber("kCos Value", 0);
//     PIDConfig.feedForward.kCos(kCosValue);

//     // This method will be called once per scheduler run
//     SmartDashboard.putNumber("Pivot encoder", p1encoder.getPosition());
//     SmartDashboard.putNumber("Pivot setpoint", pivotPID.getSetpoint());
//     SmartDashboard.putNumber("PID out", pivotIntake1.getAppliedOutput());
//     SmartDashboard.putNumber("follower moter out", pivotIntake2.get());

//   }

//   public void runIntake(double speed) {
//    if (Math.abs(p1encoder.getPosition()) > Math.abs(Constants.Manipulator.intakeDownRotations/2))
//    { 
//     Roller1.set(speed);
//     SmartDashboard.putBoolean("Rollers Active", true);
//   }
//   else {
//     Roller1.set(0);
//     SmartDashboard.putBoolean("Rollers Active", false);
//   }
//   }

//   public void motorPoseSet(double PoseRotations) {

//     pivotPID.setSetpoint(PoseRotations, SparkBase.ControlType.kPosition);
//   }

// }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.










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
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSub extends SubsystemBase {

  private final SparkMax Roller1 = new SparkMax(Constants.CANBus.RollerIntake1, MotorType.kBrushless);

  private SparkMaxConfig R1Config = new SparkMaxConfig();

  private final SparkMax Roller2 = new SparkMax(Constants.CANBus.RollerIntake2, MotorType.kBrushless);

  private SparkMaxConfig R2Config = new SparkMaxConfig();

  private final SparkMax Intake1 = new SparkMax(Constants.CANBus.pivotIntake1, MotorType.kBrushless);

  private SparkMaxConfig p1Config = new SparkMaxConfig();

private final SparkMax Intake2 = new SparkMax(Constants.CANBus.pivotIntake2, MotorType.kBrushless);

  private SparkMaxConfig p2Config = new SparkMaxConfig();

  public final RelativeEncoder p1encoder = Intake1.getEncoder();
  private double pastCurrentOutput;
  private double pastPosition;
  private double tripOutput;
  public boolean trip;
  public boolean limitswitchpressed;
  public double current;
  public DigitalInput limSwitchInput;

  public IntakeSub() {

    p1Config.smartCurrentLimit(20);
    Intake1.configure(p1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    p2Config.follow(Intake1, true);
    Intake2.configure(p2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    R1Config.smartCurrentLimit(20);
    Roller1.configure(R1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    R2Config.follow(Roller1, true);
    Roller2.configure(R2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    limSwitchInput = new DigitalInput(Constants.Manipulator.LimSwitchInput);

    // TODO, CHECK IF GEARBOX MATCHES INVERTED CONFIGURATION
  }

  @Override
  public void periodic() {
    if (Intake1.getOutputCurrent() > Constants.Manipulator.CurrentLim) {
      tripOutput = Intake1.get();
      trip = true;
    } else {
      trip = false;
    } // TODO lim switch should be wired to normally open
    if (limSwitchInput.get()) {
      limitswitchpressed = true;
    } else {
      limitswitchpressed = false;
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot encoder", p1encoder.getPosition());
     SmartDashboard.putNumber("Output Current", Roller1.getOutputCurrent());

    // pastCurrentOutput = Intake1.getOutputCurrent();
    // pastPosition = p1encoder.getPosition();

  }

  public void runIntake(double intakeSpeed, double rollerSpeed) {
    if(trip){
      Intake1.set(0);
       Roller1.set(0);
    }
    else if (((intakeSpeed > 0) && p1encoder.getPosition() >= Constants.Manipulator.maxIntakePose)) {
      Intake1.set(0);
     

    }
    else if (((intakeSpeed < 0) && p1encoder.getPosition() <= Constants.Manipulator.minIntakePose)){
         Intake1.set(0);
          Roller1.set(0);
    }
else{
  Intake1.set(intakeSpeed);
   Roller1.set(rollerSpeed);
}
}

  // private final SparkMax Roller1 = new SparkMax(Constants.CANBus.Intake1, MotorType.kBrushless);
  // private final SparkMax pivotIntake1 = new SparkMax(Constants.CANBus.pivotIntake1, MotorType.kBrushless);
  // private final SparkMax pivotIntake2 = new SparkMax(Constants.CANBus.pivotIntake2, MotorType.kBrushless);
  // private SparkMaxConfig p2Config = new SparkMaxConfig();
  // private SparkMaxConfig p1Config = new SparkMaxConfig();
  // // private ClosedLoopConfig PIDConfig = new ClosedLoopConfig();
  // private final RelativeEncoder p1encoder = pivotIntake1.getEncoder();
  // // private final SparkClosedLoopController pivotPID = pivotIntake1.getClosedLoopController();
  // // private double kCosValue = 0;

  /** Creates a new IntakeSub. */
  // public IntakeSub() {
    // PIDConfig.feedForward.kCosRatio(0);
    // kCosValue = SmartDashboard.getNumber("kCos Value", 0);
    // PIDConfig.pid(Constants.Manipulator.pivotProportion, Constants.Manipulator.pivotIntegral,
    //     Constants.Manipulator.pivotDerivative).outputRange(-1, 1);
    // p1Config.apply(PIDConfig);
    // p1Config.smartCurrentLimit(Constants.Manipulator.maxPivotCurrentLimit);
    // pivotIntake1.configure(p1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // p2Config.follow(pivotIntake1, true);
    // p2Config.smartCurrentLimit(Constants.Manipulator.maxPivotCurrentLimit);
    // // p2Config.apply(PIDConfig);
    // pivotIntake2.configure(p2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // @Override
  // public void periodic() {
  //   // kCosValue = SmartDashboard.getNumber("kCos Value", 0);
  //   // PIDConfig.feedForward.kCos(kCosValue);

  //   // This method will be called once per scheduler run
  //   SmartDashboard.putNumber("Pivot encoder", p1encoder.getPosition());
  //   // SmartDashboard.putNumber("Pivot setpoint", pivotPID.getSetpoint());
  //   SmartDashboard.putNumber("PID out", pivotIntake1.getAppliedOutput());
  //   SmartDashboard.putNumber("follower moter out", pivotIntake2.get());

  // }

  // public void runIntake(double speed) {
  //   if (Math.abs(p1encoder.getPosition()) > Math.abs(Constants.Manipulator.intakeDownRotations / 2)) {
  //     Roller1.set(speed);
  //     SmartDashboard.putBoolean("Rollers Active", true);
  //   } else {
  //     Roller1.set(0);
  //     SmartDashboard.putBoolean("Rollers Active", false);
  //   }
  //   Intake1.set(intakeSpeed);

  //   if (p1encoder.getPosition() > Constants.Manipulator.minPoseForRollerAndLimSwitch) {
  //     Roller1.set(rollerSpeed);
  //   } else {
  //     Roller1.set(0);
  //   }
  // }


  // maybe do this only when the thing is retracting by controller?
//   public void zeroEncoder() {
//     if (/*
//          * p1encoder.getPosition()<Constants.Manipulator.minPoseForRollerAndLimSwitch &&
//          */ limitswitchpressed)
//       p1encoder.setPosition(0);
//   }
// }








// public final boolean trip(){
// current = Intake1.getOutputCurrent();
// if (current >= Constants.Manipulator.CurrentLim){
// return true;
// }
// return false;
// }

//   public void pivotSpeed(double PSpeed) {
//     pivotIntake1.set(PSpeed);
//    // pivotPID.setSetpoint(PoseRotations, SparkBase.ControlType.kPosition);
//   }

//   public RelativeEncoder encoder() {
//     return p1encoder;
//   }
//   public void stopPivotMotor() {
//     pivotIntake1.set(0);
   
//   }
//   //  public SparkClosedLoopController pivot (){
//   //     return pivotPID;
//   //   }
//     public void resetEncoder() {
//       p1encoder.setPosition(0);

//     }
//       public SparkMax PivotMotor() {
   
//    return pivotIntake1;
//   }

// }

// /*
//  * // Copyright (c) FIRST and other WPILib contributors.
//  * // Open Source Software; you can modify and/or share it under the terms of
//  * // the WPILib BSD license file in the root directory of this project.
//  * 
//  * package frc.robot.subsystems;
//  * 
//  * import com.revrobotics.PersistMode;
//  * import com.revrobotics.RelativeEncoder;
//  * import com.revrobotics.ResetMode;
//  * import com.revrobotics.spark.SparkBase;
//  * import com.revrobotics.spark.SparkClosedLoopController;
//  * import com.revrobotics.spark.SparkLowLevel.MotorType;
//  * import com.revrobotics.spark.SparkMax;
//  * import com.revrobotics.spark.config.ClosedLoopConfig;
//  * import com.revrobotics.spark.config.SparkMaxConfig;
//  * 
//  * import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//  * import edu.wpi.first.wpilibj2.command.SubsystemBase;
//  * import frc.robot.utils.Constants;
//  * 
//  * public class IntakeSub extends SubsystemBase {
//  * 
//  * private final SparkMax Intake1 = new SparkMax(Constants.CANBus.pivotIntake1,
//  * MotorType.kBrushless);
//  * private final SparkMax Intake2 = new SparkMax(Constants.CANBus.pivotIntake2,
//  * MotorType.kBrushless);
//  * private SparkMaxConfig p2Config = new SparkMaxConfig();
//  * private SparkMaxConfig p1Config = new SparkMaxConfig();
//  * private final RelativeEncoder p1encoder = Intake1.getEncoder();
//  * private double pastCurrentOutput;
//  * private double pastPosition;
//  * private boolean trip;
//  * private double tripOutput;
//  * 
//  * public IntakeSub() {
//  * 
//  * p1Config.smartCurrentLimit(20);
//  * Intake1.configure(p1Config, ResetMode.kNoResetSafeParameters,
//  * PersistMode.kPersistParameters);
//  * 
//  * p2Config.follow(Intake1, false);
//  * Intake2.configure(p2Config, ResetMode.kNoResetSafeParameters,
//  * PersistMode.kPersistParameters);
//  * // TODO set inverted to false with new gearbox (DONE)!!
//  * }
//  * 
//  * @Override  
//  * public void periodic() {
//  * if (pastPosition != p1encoder.getPosition()) {
//  * if (Math.abs((Intake1.getOutputCurrent() - pastCurrentOutput)
//  * / (p1encoder.getPosition() - pastPosition)) >
//  * Constants.Manipulator.currentDerivLim) {
//  * tripOutput = Intake1.get();
//  * trip = true;
//  * }
//  * 
//  * }
//  * 
//  * // This method will be called once per scheduler run
//  * SmartDashboard.putNumber("Pivot encoder", p1encoder.getPosition());
//  * 
//  * pastCurrentOutput = Intake1.getOutputCurrent();
//  * pastPosition = p1encoder.getPosition();
//  * 
//  * }
//  * 
//  * public void runIntake(double speed) {
//  * if (!(((speed / tripOutput) > 0) && trip)) {
//  * Intake1.set(speed);
//  * trip = false;
//  * } else {
//  * Intake1.set(0);
//  * }
//  * }
//  * 
//  * }
//  */

