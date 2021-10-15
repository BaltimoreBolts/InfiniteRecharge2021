/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/* 
** PURPOSE: This is the autonmous drive commmand. Drives robot 
** forward/backward until certain distance has been travelled
** STATUS: Tested and works (signs of encoders, motor speeds should all be correct)
*/ 
 
public class AutonomousDrive extends CommandBase {
  /**
   * Creates a new Autonomous.
   */
  double mSpeed = 0; 
  double mDesiredLeftPosition = 0;
  double mDesiredRightPosition = 0;
  int mCurrentLeftPosition = 0;
  int mCurrentRightPosition = 0;
  int mInitialRightPosition = 0;
  int mInitialLeftPosition= 0;
  double mDistToTravel_in = 0;

  DriveTrain roboDT;

  public AutonomousDrive(DriveTrain robotDT, double inchesToTravel) {
    // Use addRequirements() here to declare subsystem dependencies.
    roboDT = robotDT;
    mDistToTravel_in = inchesToTravel;
     // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboDT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roboDT.resetEncoders();
    
    // // Reset the variables so our desired change in position is the same 
    // // everytime and not (desiredPosition) * numberOfCycles
    // mInitialLeftPosition = (int)roboDT.getLeftPosition();
    // mInitialRightPosition = (int)roboDT.getRightPosition();
    // mDesiredLeftPosition = 0;
    // mDesiredRightPosition = 0;
    // mInitialLeftPosition = 0;
    // mInitialRightPosition = 0;
    // roboDT.driveDistance(mDistToTravel_in);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     roboDT.closedLoopArcadeDrive(0, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roboDT.stopDT();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Left encoder counts down so that's why we're doing subtraction 
    // mDesiredLeftPosition = mInitialLeftPosition - roboDT.inchesToCounts(mDistToTravel_in, 
    // Constants.GenConstants.REV_ENCODER_CPR);
    // mCurrentLeftPosition = (int)roboDT.getLeftPosition();
   
    // // Right counts up so you do the addition 
    // mDesiredRightPosition = roboDT.inchesToCounts(mDistToTravel_in, 
    // Constants.GenConstants.REV_ENCODER_CPR) + mInitialRightPosition;
    // mCurrentRightPosition = (int)roboDT.getRightPosition();
    
    // SmartDashboard.putNumber("Desired Left", mDesiredLeftPosition);
    // SmartDashboard.putNumber("Desired Right", mDesiredRightPosition);
    // SmartDashboard.putNumber("Current Left", mCurrentLeftPosition);
    // SmartDashboard.putNumber("Current Right", mCurrentRightPosition);
    // SmartDashboard.putNumber("Initial Left", mInitialLeftPosition);
    // SmartDashboard.putNumber("Initial Right", mInitialRightPosition);

    // // We drive forward until the right and left encoders reach at or past the desired position
    // if ((mCurrentRightPosition >= mDesiredRightPosition) && (mCurrentLeftPosition<=mDesiredLeftPosition)){
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }
}
