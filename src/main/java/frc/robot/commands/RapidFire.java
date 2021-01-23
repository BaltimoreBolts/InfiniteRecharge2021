/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class RapidFire extends CommandBase {
  Indexer indexer;
  boolean[] indexerArray;

  /**
   * Creates a new RapidFire.
   */
  public RapidFire(Indexer inputIndexer) {
    indexer = inputIndexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerArray = indexer.getPCArray();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.Movement(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.Movement(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (indexerArray[0] == false && indexerArray[1]== false && indexerArray[2]== false && indexerArray[3]== false) {
      return true;
    } else {
      return false;
    } 
  }
}
