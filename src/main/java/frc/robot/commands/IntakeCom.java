// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;
import frc.robot.utils.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCom extends Command {
  private final IntakeSub m_intakeSub;
  private final double m_PivotSpeed;
  
//RyAn Wsa HerE
  /** Creates a new ManualIntakeCom. */
  public IntakeCom(IntakeSub intakeSub, double PivotSpeed) {
    m_intakeSub = intakeSub;
    m_PivotSpeed = PivotSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSub);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  


    
    // if (Math.abs(m_targetPose) >= Math.abs (Constants.Manipulator.intakeDownRotations)) m_intakeSub.runIntake(1);
    // else m_intakeSub.runIntake(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
 //   if (Math.abs(m_targetPose) >= Math.abs(Constants.Manipulator.intakeDownRotations)){
       /*m_intakeSub.runIntake(1); 
       if (Math.abs(m_intakeSub.encoder().getPosition()) < Math.abs(Constants.Manipulator.intakeDownRotations) )
       { m_intakeSub.pivotSpeed(m_PivotSpeed);
       }*/

    }
    // else {
    //   m_intakeSub.runIntake(0);}

    /*0if ( 
     Math.abs(m_intakeSub.encoder().getPosition()) < Math.abs(Constants.Manipulator.intakeDownRotations)
      && !(m_intakeSub.pivot().getSetpoint() == Constants.Manipulator.intakeUpRotations)){
   m_intakeSub.stopPivotMotor();
   //TODO, make sure above conditional doesn't stop motor whenw e actually want it to go down
    }
   if ( (Math.abs(m_intakeSub.PivotMotor().getOutputCurrent()) > Constants.Manipulator.maxPivotCurrentLimit) 
   && (Math.abs(m_intakeSub.encoder().getPosition()) < Math.abs ((Constants.Manipulator.intakeDownRotations))/9 )) {
    m_intakeSub.resetEncoder();
   }
*/
  
   //TODO make sure above conditional works with intake
 // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSub.runIntake(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
