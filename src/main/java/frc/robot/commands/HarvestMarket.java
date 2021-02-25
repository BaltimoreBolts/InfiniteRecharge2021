/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Indexer;

/*
** PURPOSE: Meant to run harvester until a PC is no longer in harvester TOF view
** STATUS: Not tested, probably not finished coding (there's no mention of mickey/front harvester motor)
*/

public class HarvestMarket extends CommandBase {
  Harvester harvestMarket;
  Indexer indexerCaptain;
  boolean isEmpty;
  boolean isFull;

  /**
   * Creates a new HarvesterDemon.
   */
  public HarvestMarket(Harvester inputHarvester, Indexer inputIndexer) {
    harvestMarket = inputHarvester;
    indexerCaptain = inputIndexer;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isEmpty = indexerCaptain.getP0();
    isFull = indexerCaptain.isIndexerFull();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isEmpty) {
      harvestMarket.setHarvesterBackSpeed(0.25);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    harvestMarket.setHarvesterBackSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (harvestMarket.getHarvesterTOF() == false){
     return true;
    } else {
     return false;
    }
  }
}
