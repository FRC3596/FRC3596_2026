// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
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
      System.out.println("The swerve did not generate; womp womp !!!!!!!!!! :(");
    }
        // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = null;
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
           m_swerve::getPose, // Robot pose supplier
           m_swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
           m_swerve::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> m_swerve.drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  // X and Y velos = meters/second, rotational velo = radians/second
  public void control(double xVelo, double yVelo, double rotVelo) {

    ChassisSpeeds velocity = new ChassisSpeeds(xVelo, yVelo, rotVelo);
    m_swerve.drive(velocity);

  }

  public Rotation2d getYaw() {
    return m_swerve.getOdometryHeading();

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler runs
  }

}
