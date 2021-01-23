/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class PowerCellSucker extends CommandBase {
  Harvester roboKirby;
  double harDir; 
  boolean section; 

  /**
   * Creates a new PowerCellSucker.
   * Negative direction sucks the power cell in 
   * true input selection is the top (near indexer) part of the harvester, false is the front part 
   */
  public PowerCellSucker(Harvester roboHarvest, double direction, boolean inputSection) {
    roboKirby = roboHarvest;
    harDir = direction;
    section = inputSection;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboKirby);
  }

  // Called when the command is initially scheduled.
  // Make sure that the Mickey motor does not move if PC in hand
  @Override
  public void initialize() {
    /*
    if (roboKirby.getHarvesterTOF() == true) {
      roboKirby.setMickeySpeed(0.0);
    } else {
      roboKirby.setMickeySpeed(0.25 * harDir);
    }
    */
    if (section) {
      roboKirby.setMinnieSpeed(0.35 * harDir);
    } else {
      roboKirby.setMickeySpeed(0.35 * harDir);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (section) {
      roboKirby.setMinnieSpeed(0.0);
    } else {
      roboKirby.setMickeySpeed(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /* 
    if (roboKirby.getHarvesterTOF() == true) {
      return true;
    } else {
      return false;
    }
    */ 
    return false; 
  }

}
