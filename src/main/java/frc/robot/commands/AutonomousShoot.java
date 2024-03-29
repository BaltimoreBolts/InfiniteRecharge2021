/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Harvester;


/* 
** PURPOSE: This is the command to shoot for autonomous mode
** STATUS: Not tested to my knowledge
*/ 

public class AutonomousShoot extends CommandBase {
  Shooter robotShooter;
  Indexer roboIndexer;
  Harvester roboHarvester;
  
  /**
   * Creates a new AutonomousShooter.
   */
  public AutonomousShoot(Shooter inputShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    robotShooter = inputShooter;
    
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new MoveShooter(robotShooter, true, 8000);
    //for (int i = 0; i <3; i++) {
      //new FirePowerCell(robotShooter, roboIndexer, roboHarvester);
    //}
    // new FirePowerCell(robotShooter, roboIndexer, roboHarvester);
    //robotShooter.SetShooterSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
