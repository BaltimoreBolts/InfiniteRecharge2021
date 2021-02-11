/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

/* 
** PURPOSE: This was a testing function to reverse indexer and spit out the power cells
** STATUS: Works!
*/

public class reverseIndexer extends CommandBase {
  Indexer roboIndexer;
  double speed = 0; 
  double currentPosition = 0;
  double desiredPosition = 0;
  double initialPosition= 0;
  double degreesToRotate = 120;
  int n = 0;
  /**
   * Creates a new moveIndexer.
   */
  public reverseIndexer(Indexer robotIndexer) {
    roboIndexer = robotIndexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotIndexer);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = roboIndexer.getdesiredSpeed();
    initialPosition = roboIndexer.getEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roboIndexer.Movement(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //The speed is being set to 0/
    roboIndexer.Movement(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    /*Upwards is positive encoder, but the speed is negative to go up*/
    //desiredPosition = -degreesToRotate/360.0+initialPosition;
    desiredPosition =  roboIndexer.getEncoderValue()-(70/3);
    
    // ... for PID comment out everything below here, replace with 
    return roboIndexer.MoveToPosition(desiredPosition); 
    // 
    /*
    currentPosition = roboIndexer.getEncoderValue();
    if (currentPosition <= desiredPosition){
      return true;
  } else {
    return false;
  }
    */
  }
}
