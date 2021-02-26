/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

/*
** PURPOSE: Meant to coordinate the moving of the indexer to shift PCs up, correctly adjust
** PC locator array so we know where PCs are located in the indexer. Also adjust any overshoot
** of the indexer.
** STATUS: Not tested. A lot of the functionality is commented out.
*/

public class IndexerCaptain extends CommandBase {
  Indexer indexerCaptain;
  boolean isEmpty;
  boolean isFull;
  double encoderValue;
  double encoderSpeed;
  double targetPosition;

  /**
   * Creates a new IndexerCaptain.
   */
  public IndexerCaptain(final Indexer inputIndexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    indexerCaptain = inputIndexer;
    // roboShooter = inputShooter;
    addRequirements(indexerCaptain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isEmpty = indexerCaptain.getP0();
    isFull = indexerCaptain.isIndexerFull();
    encoderValue = indexerCaptain.getEncoderValue();
    targetPosition = encoderValue + (1.0 / 3.0);

    indexerCaptain.MoveToPosition(targetPosition, 0);
    indexerCaptain.setIndexerSpeed(-0.15);
    // indexerCaptain.ResetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // indexerCaptain.Movement(-0.15);
    /**
     * if (!isFull || roboShooter.getReadyToFire()) { // Shift the PC's up one level
     * indexerCaptain.Movement(-0.25); }
     **/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    indexerCaptain.ShiftPCArray(true);
    indexerCaptain.setIndexerSpeed(0);
    indexerCaptain.CalculateOvershoot(indexerCaptain.getEncoderValue(), targetPosition);
    // roboShooter.SetShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /**
    if (indexerCaptain.getEncoderValue() <= (targetPosition - indexerCaptain.GetOvershoot())) {
      return false;
    } else {
      return true;
    }
    **/

    return indexerCaptain.MoveToPosition(targetPosition, 0);
  }
}
