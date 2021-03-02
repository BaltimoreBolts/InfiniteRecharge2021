/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

/**
 * PURPOSE: This was a testing function to exclusively move the indexer up one.
 * STATUS: Works!
 */
public class MoveIndexer extends CommandBase {
  Indexer roboIndexer;
  double indexerSpeed = 0;
  boolean direction = true; // default to moving up
  boolean atDesiredPosition = false;
  double currentPosition = 0;
  double desiredPosition = 0;
  double initialPosition = 0;
  double degreesToRotate = 120;
  double startingPos = 0;
  double resetDistance = 0;
  double startTime = 0;
  final double maxPIDduration = 1e9; // 1 second in nano seconds

  /**
   * Creates a new moveIndexer.
   */
  public MoveIndexer(Indexer roboIndexer, boolean direction) {
    this.roboIndexer = roboIndexer;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (direction) {
      indexerSpeed = -roboIndexer.getDesiredSpeed(); // negative to move updward
    } else {
      indexerSpeed = roboIndexer.getDesiredSpeed();
    }

    initialPosition = roboIndexer.getEncoderValue();
    startingPos = roboIndexer.getAbsEncoderValue() + 0.12;

    // Calculate reset distance, slighly up or down to achieve desired starting configuration
    double mod_math = startingPos - Math.floor(startingPos/(1.0/3.0)) * (1.0/3.0);
    if (mod_math > 0.1666) {
      resetDistance = (1.0/3.0 - mod_math);
    } else {
      resetDistance = (-mod_math);
    }

    SmartDashboard.putNumber("[Indexer] Mod Math", mod_math);
    SmartDashboard.putNumber("[Indexer] Reset Distance", resetDistance);

    // start a timer to check later if we exceed 1 second for a rotation
    startTime = System.nanoTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roboIndexer.setIndexerSpeed(indexerSpeed);
    atDesiredPosition = roboIndexer.moveToPosition(desiredPosition, resetDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roboIndexer.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Upwards is positive encoder, but the speed is negative to go up
    if (direction) {
      desiredPosition = initialPosition - (70.0/3.0);
    } else {
      desiredPosition = initialPosition + (70/3.0);
    }

     // return if at the right value or too long of a duration
    double duration = System.nanoTime() - startTime;
    return atDesiredPosition || duration > maxPIDduration;
  }
}
