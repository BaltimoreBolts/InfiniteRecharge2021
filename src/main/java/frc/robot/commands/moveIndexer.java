/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

/**
 * PURPOSE: This was a testing function to exclusively move the indexer up one.
 * Also calls inner harvester motor so PC doesn't get stuck in the first slot
 * STATUS: Works!
 */
public class MoveIndexer extends CommandBase {
  Indexer roboIndexer;
  // Harvester roboHarvester;
  double indexerSpeed = 0;
  boolean direction = true; // default to moving up
  // double HarvesterSpeed = 0;
  double currentPosition = 0;
  double desiredPosition = 0;
  double initialPosition = 0;
  double degreesToRotate = 120;
  double startingPos = 0;
  double resetDistance = 0;
  // double startTime = 0;
  // final double maxPIDduration = 1e9; // 1 second in nano seconds


  /**
   * Creates a new moveIndexer.
   */
  public MoveIndexer(Indexer robotIndexer, boolean direction) { // , Harvester robotHarvester) {
    roboIndexer = robotIndexer;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (direction) {
      indexerSpeed = -roboIndexer.getDesiredSpeed();
    } else {
      indexerSpeed = roboIndexer.getDesiredSpeed();
    }

    initialPosition = roboIndexer.getEncoderValue();
    startingPos = roboIndexer.getAbsEncoderValue() + 0.12;

    // Move indexer slighly up or down to achieve desired starting configuration
    double mod_math = startingPos - Math.floor(startingPos/(1.0/3.0)) * (1.0/3.0);
    if (mod_math > 0.1666) {
      resetDistance = (1.0/3.0 - mod_math);
    } else {
      resetDistance = (-mod_math);
    }

    // startTime = System.nanoTime();
    SmartDashboard.putNumber("Mod Math", mod_math);
    SmartDashboard.putNumber("Reset Distance", resetDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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

    // ... for PID comment out everything below here, replace with
    // double duration = System.nanoTime() - startTime;

    return roboIndexer.MoveToPosition(desiredPosition, resetDistance); // || duration > maxPIDduration;
  }
}
