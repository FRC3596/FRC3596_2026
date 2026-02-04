// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
//bob was here 1/15

public class SwerveSub extends SubsystemBase {
  SwerveDrive m_swerve;

  /** Creates a new SwerveSub. */
  public SwerveSub() {
    final double maximumSpeed = Units.feetToMeters(4.5);
    File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      //loki wSA hare 1/17/26
      m_swerve = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      System.out.println("The swerve did not generate; womp womp !!!!!!!!!!!!! :( :( :( ");
    }
  }
//X and Y velos = meters/second, rotational velo = radians/second
  public void control(double xVelo, double yVelo, double rotVelo) {

    ChassisSpeeds velocity = new ChassisSpeeds(yVelo, xVelo, rotVelo); //(considering adding 2.22 multiplyer to rotvelo)
    m_swerve.drive(velocity);

  }
public Rotation2d getYaw(){
return m_swerve.getOdometryHeading();

}

  @Override
  public void periodic() {

    // This method will be called once per scheduler runs
  }
}
