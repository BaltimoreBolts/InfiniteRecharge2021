/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Harvester;
import frc.robot.Globals.PCArray;


/**
 * PURPOSE: This is the sequential command to run the harvester, wait until it detects the ball, 
 * and moves the indexer up, updating the PCArray
 * STATUS: Never tested
 */
public class Intake extends SequentialCommandGroup {
   // Creates a new FirePowerCell
  public Intake(Indexer roboIndexer, Harvester roboHarvester) {
    super (
        new MoveIndexer(roboIndexer, false, PCArray.getLowestPCPos()), // pre-move down to lowest position
        new MoveHarvester(roboHarvester, true), // then run the harvester
        new ParallelDeadlineGroup( // once a ball is detected, move the indexer up one
        // deadline group will end when moveIndexer is done
            new MoveIndexer(roboIndexer, true, 1), 
            new MoveHarvester(roboHarvester, true)
        ) 
    );
    PCArray.intakePC(); // set lowest index to true
  }
}
