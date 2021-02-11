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
import frc.robot.subsystems.Harvester;

/* 
** PURPOSE: This was a testing function to exclusively move the indexer up one. 
** Also calls inner harvester motor so PC doesn't get stuck in the first slot
** STATUS: Works!
*/
public class moveIndexer extends CommandBase {
  Indexer roboIndexer;
  Harvester roboHarvester;
  double IndexerSpeed = 0; 
  double HarvesterSpeed = 0;
  double currentPosition = 0;
  double desiredPosition = 0;
  double initialPosition= 0;
  double degreesToRotate = 120;
  int n = 0;
  /**
   * Creates a new moveIndexer.
   */
  public moveIndexer(Indexer robotIndexer, Harvester robotHarvester) {
    roboIndexer = robotIndexer;
    roboHarvester = robotHarvester;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotIndexer);
    addRequirements(robotHarvester);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This is negative to move the indexer up
    IndexerSpeed = -roboIndexer.getdesiredSpeed();
    HarvesterSpeed = roboHarvester.getBackDesiredSpeed();
    initialPosition = roboIndexer.getEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //roboHarvester.setMinnieSpeed(HarvesterSpeed);
    //roboIndexer.Movement(IndexerSpeed); // for PID comment this out and add ...
    //roboIndexer.SetIndexerSpeedPID(); // ???
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //The speed is being set to 0/
    roboIndexer.Movement(0);
    roboHarvester.setMinnieSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {


    /*Upwards is positive encoder, but the speed is negative to go up*/
    //desiredPosition = degreesToRotate/360.0+initialPosition;
    //desiredPosition = roboIndexer.getEncoderValue() + (70.0/3.0);
    desiredPosition = 1;
    // ... for PID comment out everything below here, replace with 
    return roboIndexer.MoveToPosition(desiredPosition); 
    // 

    //return roboIndexer.MoveToPosition(desiredPosition);
    /* currentPosition = roboIndexer.getEncoderValue();
     if (currentPosition >= desiredPosition){
       return true;
   } else {
     return false;
   }*/
    
  }
}
