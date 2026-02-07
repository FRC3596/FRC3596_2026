// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawDetection;
import frc.robot.utils.LimelightHelpers.RawFiducial;

public class LimeLightSub extends SubsystemBase {
  private SwerveSub m_swerveSub;
  boolean doRejectUpdate = false;
  private PIDController VisionHorizontalPID = new PIDController(Constants.LimeLight.VisionPID.LimelightHorizontal_P,
      Constants.LimeLight.VisionPID.LimelightHorizontal_I, Constants.LimeLight.VisionPID.LimelightHorizontal_D);
  private PIDController VisionVerticalPID = new PIDController(Constants.LimeLight.VisionPID.LimelightVertical_P,
      Constants.LimeLight.VisionPID.LimelightVertical_I, Constants.LimeLight.VisionPID.LimelightVertical_D);
  private PIDController VisionRotationPID = new PIDController(Constants.LimeLight.VisionPID.LimelightRotation_P,
      Constants.LimeLight.VisionPID.LimelightRotation_I, Constants.LimeLight.VisionPID.LimelightRotation_D);
  private double VisionHorizontalPIDSpeed = 0; 
  private double VisionVerticalPIDSpeed = 0;
  private double VisionRotationPIDSpeed = 0; 
  private double desiredPose = 0;
  
  /** Creates a new LimeLightSub. */
  public LimeLightSub(SwerveSub swerveSub) {
    m_swerveSub = swerveSub;
    LimelightHelpers.setCameraPose_RobotSpace("",
        Constants.LimeLight.ForwardOffset, // Forward offset (meters)
        Constants.LimeLight.SideOffset, // Side offset (meters)
        Constants.LimeLight.HeightOffset, // Height offset (meters)
        Constants.LimeLight.RollOffset, // Roll (degrees)
        Constants.LimeLight.PitchOffset, // Pitch (degrees)
        Constants.LimeLight.YawOffset // Yaw (degrees)
    );

    LimelightHelpers.setLEDMode_ForceOff("");
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[]{1, 2, 3, 4}); // Only track these tag IDs
    SmartDashboard.putNumber("limelight Despired pose", 0);
  }

  @Override
  public void periodic() {
        // Basic targeting data
    double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC(""); // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC(""); // Vertical offset from principal pixel/point to target in degrees

    // First, tell Limelight your robot's current orientation
    double robotYaw = SwerveSub.robotYaw;
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    // Add it to your pose estimator
    m_swerveSub.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5,
        9999999));
    if(hasTarget){
    m_swerveSub.addVisionMeasure(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds);
    // This method will be called once per scheduler run
    

    //keep an eye on the "SwerveSub.publicSwerve.getYaw().getDegrees()" part may not be what we need
    LimelightHelpers.SetRobotOrientation("limelight", m_swerveSub.getGyroDeg(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // if our angular velocity is greater than 360 degrees per second, ignore vision
    // updates
    //keep an eye on this value it may or may not be what we need
    if (Math.abs(m_swerveSub.getAngularVelocity()) > 360) {
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      m_swerveSub.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      m_swerveSub.addVisionMeasure(
          mt2.pose,
          mt2.timestampSeconds);
     }

    VisionHorizontalPIDSpeed = VisionHorizontalPID.calculate(mt2.pose.getX());
     VisionVerticalPIDSpeed = VisionVerticalPID.calculate(mt2.pose.getY());
     VisionRotationPIDSpeed = VisionRotationPID.calculate(mt2.pose.getRotation().getDegrees());


     SmartDashboard.putNumber("LimeLight Pose X", mt2.pose.getX());
     SmartDashboard.putNumber("LimeLight Pose Y", mt2.pose.getY());
     SmartDashboard.putNumber("LimeLight Pose Theta", mt2.pose.getRotation().getDegrees());
     SmartDashboard.putNumber("Limelight Tag ID", (int) LimelightHelpers.getFiducialID(""));

     if((mt2.pose.getX() <= 0 ) && (mt2.pose.getY() == 0) && (mt2.pose.getRotation().getDegrees() == 0)) {
      SmartDashboard.putString("Ready to Shoot", "FIRE!!!!");
     }
     else {
      SmartDashboard.putString("Ready to Shoot", "HOLD");
     }
    }
    else {
      // deafault if no target detected
     desiredPose = SmartDashboard.getNumber("limelight Despired pose", 9);
     SmartDashboard.putNumber("LimeLight Pose X", -99);
     SmartDashboard.putNumber("LimeLight Pose Y", -99);
     SmartDashboard.putNumber("LimeLight Pose Theta", -99);
     SmartDashboard.putNumber("Limelight pose wanted", desiredPose);
     VisionHorizontalPIDSpeed = 0;
     VisionVerticalPIDSpeed = 0;
     VisionRotationPIDSpeed = 0;
    }
  }
  public void addVisionHorizontalSetpoint(double setpoint) {
    VisionHorizontalPID.setSetpoint(setpoint);
  }
  public void addVisionVerticalSetpoint(double setpoint) {
    VisionVerticalPID.setSetpoint(setpoint);
  }
  public void addVisionRotationSetpoint(double setpoint) {
    VisionRotationPID.setSetpoint(setpoint);
  }
  public double CalculateVisionHorizontalPID(double setpoint) {
    return VisionHorizontalPIDSpeed;
  }
  public double CalculateVisionVerticalPID(double setpoint) {
    return VisionVerticalPIDSpeed;
  }
  public double CalculateVisionRotationPID(double setpoint) {
    return VisionRotationPIDSpeed;
  }
  public boolean hasTarget() {
    return LimelightHelpers.getTV("");
  }
  public boolean atSetPoint() {
    return VisionHorizontalPID.atSetpoint() && VisionVerticalPID.atSetpoint() && VisionRotationPID.atSetpoint();
  }
  public int getTargetID() {
    return (int) LimelightHelpers.getFiducialID("");
  }
}