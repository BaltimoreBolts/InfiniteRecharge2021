/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

 
public class AutonomousTurn extends CommandBase {
  /**
   * Creates a new Autonomous.
   */

  double mArcAngleToTravel_deg = 0;
  double mRadius = 0;
  boolean mDirection = true;
 
   
  DriveTrain roboDT;
  public AutonomousTurn(DriveTrain robotDT, double radius, double angle, boolean clockwise) {
    // Use addRequirements() here to declare subsystem dependencies.
    roboDT = robotDT;
    mArcAngleToTravel_deg = angle;
    mRadius = radius;
    mDirection = clockwise;
     // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboDT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roboDT.resetEncoders();
    // roboDT.autonTurn(mRadius, mArcAngleToTravel_deg, mDirection);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roboDT.stopDT();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return roboDT.autonTurn(mRadius, mArcAngleToTravel_deg, mDirection);
    return false;
  }
}
