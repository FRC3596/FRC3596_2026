// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
//bob was here 1/15

public class SwerveSub extends SubsystemBase {
  private SwerveDrive m_swerve;

  /** Creates a new SwerveSub. */
  public SwerveSub() {
    final double maximumSpeed = Units.feetToMeters(4.5);
    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      //loki wSA hare 1/17/26
      m_swerve = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      System.out.println("The swerve did not generate; womp womp !!!!!!!!!! :(");
    }
  }
//X and Y velos = meters/second, rotational velo = radians/second
  public void control(double xVelo, double yVelo, double rotVelo) {

    ChassisSpeeds velocity = new ChassisSpeeds(yVelo, xVelo, rotVelo);
    m_swerve.drive(velocity);
    
  }

  public double getGyroDeg() {

  return m_swerve.getYaw().getDegrees();
  }

  public void addVisionMeasure(Pose2d botpose, double timeSeconds) {
    m_swerve.addVisionMeasurement(botpose, timeSeconds);

  }
  public double getAngularVelocity() {
    return m_swerve.getGyro().getYawAngularVelocity().magnitude();
  }
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> stdDevs) {
    m_swerve.setVisionMeasurementStdDevs(stdDevs);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler runs
  }
}
