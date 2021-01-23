/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootPowerCell extends CommandBase {
  Shooter roboShooter;
  double calculatedRPM = 0;
  
  /**
   * Creates a new ShootPowerCell.
   */
  public ShootPowerCell(Shooter inputShooter) {
    roboShooter = inputShooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calculatedRPM = roboShooter.getNeededRPM();
    //calculatedRPM = -0.15; // Comment this in to set speed directly
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //roboShooter.PIDTuner();
    System.out.println("\n\nSetting shooter speed\n\n");
    roboShooter.SetShooterSpeed(calculatedRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roboShooter.setReadyToFire(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return roboShooter.AtSpeed(calculatedRPM);
    return true;
  }
}
