/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals.PCArray;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants.IndexerConstants;

/**
 * PURPOSE: This was a testing function to exclusively move the indexer up one.
 * STATUS: Works!
 */
public class MoveIndexer extends CommandBase {
  Indexer roboIndexer;
  double indexerSpeed = 0;
  boolean direction = true; // default to moving up
  boolean mAtDesiredPosition = false;
  double currentPosition = 0;
  double mDesiredPosition = 0;
  double mInitialPosition = 0;
  double degreesToRotate = 120;
  double mStartingPos = 0;
  double mResetDistance = 0;
  double startTime = 0;
  int numPos = 0;
  /**
   * Creates a new moveIndexer.
   */
  public 
  MoveIndexer(Indexer roboIndexer, boolean direction, int num_pos) {
    this.roboIndexer = roboIndexer;
    this.direction = direction;
    this.numPos = (num_pos == -1) ? 0 : num_pos; // if we are told that there are no PCs, dont move
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboIndexer);
  }

  /**
   * Overloading for defualts: if no params passed, move up one. if no number passed, move one
   */
  public MoveIndexer(Indexer roboIndexer) {
    this(roboIndexer, true, 1);
  }

  public MoveIndexer(Indexer roboIndexer, boolean direction) {
    this(roboIndexer, direction, 1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (direction) {
      // indexerSpeed = -roboIndexer.getDesiredSpeed(); // negative to move updward
      indexerSpeed = -0.5;
    } else {
      // indexerSpeed = roboIndexer.getDesiredSpeed();
      indexerSpeed = 0.5;
    }
    findHomingOffset();
    // start a timer to check later if we exceed 1 second for a rotation
    startTime = System.nanoTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roboIndexer.setIndexerSpeed(indexerSpeed);
    // mAtDesiredPosition = roboIndexer.moveToPosition(numPos * mDesiredPosition, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roboIndexer.setIndexerSpeed(0);
    
    if (direction) {
      PCArray.moveUp();
    } else {
      PCArray.moveDown();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Upwards is positive encoder, but the speed is negative to go up
    // is  this being used?
    if (direction) {
      mDesiredPosition = mInitialPosition - (70.0/3.0);
    } else {
      mDesiredPosition = mInitialPosition + (70/3.0);
    }

     // return if at the right value or too long of a duration
    double duration = System.nanoTime() - startTime;
    // return mAtDesiredPosition || duration > IndexerConstants.kMaxPIDduration;
    return false;
  }

  private void findHomingOffset(){
    mInitialPosition = roboIndexer.getEncoderValue();
    mStartingPos = roboIndexer.getAbsEncoderValue();

    // Calculate reset distance, slighly up or down to achieve desired starting configuration
    double mod_math = mStartingPos - Math.floor(mStartingPos/(1.0/3.0)) * (1.0/3.0);
    if (mod_math > 0.1666) {
      mResetDistance = (1.0/3.0 - mod_math);
    } else {
      mResetDistance = (-mod_math);
    }

    SmartDashboard.putNumber("[Indexer] Mod Math", mod_math);
    SmartDashboard.putNumber("[Indexer] Reset Distance", mResetDistance);
  }
}
