/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;


/**
 * PURPOSE: This is the sequential command to run the harvester, wait until it detects the ball, 
 * and moves the indexer up, updating the PCArray
 * STATUS: Never tested
 */
public class Shoot extends SequentialCommandGroup {
   // Creates a new FirePowerCell
  public Shoot(Indexer roboIndexer, Shooter roboShooter) {
    super (
        new MoveShooter(roboShooter, true, roboShooter.getDesiredRPM()), // should finish when at RPM
        new ParallelDeadlineGroup(
            new MoveIndexer(roboIndexer, true, roboIndexer.getHighestPCPos()),
            new MoveShooter(roboShooter, true, roboShooter.getDesiredRPM())
        )
    );
  }
}
