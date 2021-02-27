/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

/**
 * PURPOSE: This raises/lowers the elevator mechanism so we can drive up and hook the switch
 * STATUS: Tested and worked
 */
public class MoveElevator extends CommandBase {
  private Elevator elevator;
  private boolean direction; // true for up, false for down

  /**
   * Creates a new Elevator.
   */
  public MoveElevator(Elevator roboElevator, boolean direction) {
    elevator = roboElevator;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (direction) {
      // Disengage ratchet and engages motor to move lift up
      elevator.disengageRatchet();
      elevator.setSpeed(0.55);
    } else {
      // Engage ratchet so the robot doesn't fall down
      elevator.engageRatchet();
      elevator.setSpeed(-0.85);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (direction) {
      // Engages ratchet again and disables motor to lock lift position
      elevator.engageRatchet();
      elevator.setSpeed(0);
    } else {
      elevator.setSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (direction) {
      return elevator.getElevatorEncoder() > 3.65;
    } else {
      return elevator.getElevatorEncoder() < 1;
    }
  }
}
