/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

/* 
** PURPOSE: This brings the elevator mechanism down (how we pull the robot up)
** STATUS: Tested and worked
*/ 

public class ElevatorGoDown extends CommandBase {
  Elevator RoboDarth;
  /**
   * Creates a new ElevatorGoDown.
   */
  public ElevatorGoDown(Elevator roboElevator) {
    RoboDarth = roboElevator;
    addRequirements(roboElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Engage ratchet so the robot doesn't fall down
    RoboDarth.engageRatchet();
    RoboDarth.setSpeed(-0.85);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  // Stop motor
  @Override
  public void end(boolean interrupted) {
    RoboDarth.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (RoboDarth.getElevatorEncoder() < 1) {
      return true;
    } else {
      return false;
    }
  }
}
