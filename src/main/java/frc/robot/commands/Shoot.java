/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.Globals.PCArray;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/**
 * PURPOSE: This is the sequential command to run the harvester, wait until it detects the ball, 
 * and moves the indexer up, updating the PCArray
 * STATUS: Never tested
 */
public class Shoot extends SequentialCommandGroup {
   // Creates a new FirePowerCell
  public Shoot(Indexer roboIndexer, Shooter roboShooter) {
    super (
        // new MoveShooter(roboShooter, true, roboShooter.getDesiredRPM()), // spin up shooter
        new MoveShooter(roboShooter, true, 8000) // spin up shooter
        //new MoveIndexer(roboIndexer, true, 3 - PCArray.getHighestPCPos()), // move indexer up to shoot
        //new WaitCommand(3), // let the shooter spin while the ball goes through the flywheel
        //new MoveShooter(roboShooter, true, 0) // set shooter to idle after shot
    );
  }
}
