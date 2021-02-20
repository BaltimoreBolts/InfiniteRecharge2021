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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

/* 
** PURPOSE: This is the sequential command to shoot power cells, move indexer up, and then harvest
** STATUS: Never tested
*/ 
public class FirePowerCell extends SequentialCommandGroup {

   // Creates a new FirePowerCell
  public FirePowerCell(Shooter roboShooter, Indexer roboIndexer, Harvester roboHarvester) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super (
        new ShootPowerCell(roboShooter),
        new MoveIndexer(roboIndexer,roboHarvester)
        // new HarvestMarket(roboHarvester, roboIndexer)
        // new IndexerHarvestMayhem(roboIndexer, roboHarvester, robotShooter) //idk what this does
    );
  }
}
