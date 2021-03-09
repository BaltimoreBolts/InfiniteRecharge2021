// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harvester;

/*
 * PURPOSE: This was a testing function to exclusively run the two harvester motors
 * STATUS: Works!
 */
public class MoveHarvester extends CommandBase {
  Harvester roboHarvester;
  double HarvesterFrontSpeed = 0.6; // these are both positive values, back should be a bit faster or equal 
  double HarvesterBackSpeed = 0.8;
  boolean direction = true; // true for intake

  /** Creates a new HarvesterIn. */
  public MoveHarvester(Harvester robotHarvester, boolean direction) {
    this.roboHarvester = robotHarvester;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotHarvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction) {
      roboHarvester.setHarvesterFrontSpeed(-HarvesterFrontSpeed);
      roboHarvester.setHarvesterBackSpeed(HarvesterBackSpeed);
    } else {
      roboHarvester.setHarvesterFrontSpeed(HarvesterFrontSpeed);
      roboHarvester.setHarvesterBackSpeed(-HarvesterBackSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop both motors
    roboHarvester.setHarvesterBackSpeed(0);
    roboHarvester.setHarvesterFrontSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return roboHarvester.getHarvesterTOF(); //need to have an override
  }
}
