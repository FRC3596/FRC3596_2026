// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelpers;

public class LimeLightSub extends SubsystemBase {
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
    // // First, tell Limelight your robot's current orientation
    // double robotYaw = m_gyro.getYaw();
    // LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    // // Get the pose estimate
    // LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    // // Add it to your pose estimator
    // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5,
    //     9999999));
    // m_poseEstimator.addVisionMeasurement(
    //     limelightMeasurement.pose,
    //     limelightMeasurement.timestampSeconds);
    // // This method will be called once per scheduler run
    // LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
    //     0, 0, 0, 0, 0);
    // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // // if our angular velocity is greater than 360 degrees per second, ignore vision
    // // updates
    // if (Math.abs(m_gyro.getRate()) > 360) {
    //   doRejectUpdate = true;
    // }
    // if (mt2.tagCount == 0) {
    //   doRejectUpdate = true;
    // }
    // if (!doRejectUpdate) {
    //   m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    //   m_poseEstimator.addVisionMeasurement(
    //       mt2.pose,
    //       mt2.timestampSeconds);
    //  }

  }
}
