// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harvester;

/*
** PURPOSE: This was a testing function to exclusively run the two harvester motors
** STATUS: Works!
*/
public class HarvesterIn extends CommandBase {
  Harvester roboHarvester;
  double HarvesterFrontSpeed = 0;
  double HarvesterBackSpeed = 0;

  /** Creates a new HarvesterIn. */
  public HarvesterIn(Harvester robotHarvester) {
    roboHarvester = robotHarvester;
    addRequirements(robotHarvester);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    HarvesterFrontSpeed = roboHarvester.getFrontDesiredSpeed();
    HarvesterBackSpeed = roboHarvester.getBackDesiredSpeed();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // A positive back motor (minnie) speed brings power cells in
    // A negative front motor (mickey) speed brings power cells in
    HarvesterBackSpeed = 0.3;
    HarvesterFrontSpeed = 0.3;
    roboHarvester.setHarvesterBackSpeed(HarvesterBackSpeed);
    roboHarvester.setHarvesterFrontSpeed(-HarvesterFrontSpeed);
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
    return false;
  }
}
