/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorGoUp extends CommandBase {
  Elevator RoboVader;
  /**
   * Creates a new ElevatorGoUp.
   */
  public ElevatorGoUp(Elevator roboElevator) {
    RoboVader = roboElevator;
    addRequirements(roboElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RoboVader.disengageRatchet();
    RoboVader.setSpeed(0.55);
  }
  // Disengages ratchet and engages motor to move lift up

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RoboVader.engageRatchet();
    RoboVader.setSpeed(0);
  }
  // Engages ratchet again and disables motor to lock lift position

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RoboVader.getElevatorEncoder() > 3.65) {
      return true;
    } else {
      return false; 
    }
  }
}
