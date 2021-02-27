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
 * PURPOSE: This was a testing function to reverse indexer and spit out the power cells
 * STATUS: Works!
 */
public class ReverseIndexer extends CommandBase {
  Indexer roboIndexer;
  double indexerSpeed = 0;
  double currentPosition = 0;
  double desiredPosition = 0;
  double initialPosition = 0;
  double degreesToRotate = 120;
  double startingPos = 0;
  double resetDistance = 0;
  int n = 0;

  /**
   * Creates a new moveIndexer.
   */
  public ReverseIndexer(Indexer robotIndexer) {
    roboIndexer = robotIndexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexerSpeed = roboIndexer.getDesiredSpeed();

    initialPosition = roboIndexer.getEncoderValue();
    startingPos = roboIndexer.getAbsEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roboIndexer.setIndexerSpeed(indexerSpeed);
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
    desiredPosition = initialPosition + (70.0/3.0);

    // ... for PID comment out everything below here, replace with
    return roboIndexer.MoveToPosition(desiredPosition, resetDistance);
  }
}
