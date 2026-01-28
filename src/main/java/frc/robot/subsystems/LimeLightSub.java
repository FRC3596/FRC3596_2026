// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelpers;

public class LimeLightSub extends SubsystemBase {
  boolean doRejectUpdate = false;
  
  /** Creates a new LimeLightSub. */
  public LimeLightSub() {

    LimelightHelpers.setCameraPose_RobotSpace("",
        Constants.LimeLight.ForwardOffset, // Forward offset (meters)
        Constants.LimeLight.SideOffset, // Side offset (meters)
        Constants.LimeLight.HeightOffset, // Height offset (meters)
        Constants.LimeLight.RollOffset, // Roll (degrees)
        Constants.LimeLight.PitchOffset, // Pitch (degrees)
        Constants.LimeLight.YawOffset // Yaw (degrees)
    );

    
    // Basic targeting data
    double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC(""); // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC(""); // Vertical offset from principal pixel/point to target in degrees

    LimelightHelpers.setLEDMode_ForceOff("");

  }

  @Override
  public void periodic() {
    // First, tell Limelight your robot's current orientation
    double robotYaw = SwerveSub.publicSwerve.getYaw().getDegrees();
    LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    // Add it to your pose estimator
    SwerveSub.publicSwerve.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5,
        9999999));
    SwerveSub.publicSwerve.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds);
    // This method will be called once per scheduler run

    //keep an eye on the "SwerveSub.publicSwerve.getYaw().getDegrees()" part may not be what we need
    LimelightHelpers.SetRobotOrientation("limelight", SwerveSub.publicSwerve.getYaw().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // if our angular velocity is greater than 360 degrees per second, ignore vision
    // updates
    //keep an eye on this value it may or may not be what we need
    if (Math.abs(SwerveSub.publicSwerve.getGyro().getYawAngularVelocity().magnitude()) > 360) {
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      SwerveSub.publicSwerve.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      SwerveSub.publicSwerve.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
     }
  }
}
