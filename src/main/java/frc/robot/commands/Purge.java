/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Harvester;

/**
 * PURPOSE: This is the sequential command to shoot power cells, move indexer up, and then harvest
 * STATUS: Never tested
 */
public class Purge extends ParallelDeadlineGroup {

   // Creates a new FirePowerCell
  public Purge(Shooter roboShooter, Indexer roboIndexer, Harvester roboHarvester) {
    super (
        new MoveHarvester(roboHarvester, true),
        new MoveIndexer(roboIndexer, false)
    );
  }
}
