/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Harvester;

/**
 * PURPOSE: This is the sequential command move indexer up and harvester at same time
 */
public class Acquire extends ParallelDeadlineGroup {

   // Creates a new FirePowerCell
  public Acquire(Indexer roboIndexer, Harvester roboHarvester) {
    super (
        new MoveHarvester(roboHarvester, true),
        new MoveIndexer(roboIndexer, true)
    );
  }
}
