/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


 
public class AutonomousDrive extends CommandBase {
  /**
   * Creates a new Autonomous.
   */
  double speed = 0; 
  double desiredLeftPosition = 0;
  double desiredRightPosition = 0;
  int currentLeftPosition = 0;
  int currentRightPosition = 0;
  int initialRightPosition = 0;
  int initialLeftPosition= 0;
  double distToTravel_in = 0;
 
   
  DriveTrain roboDT;
  public AutonomousDrive(DriveTrain robotDT, double inchesToTravel) {
    // Use addRequirements() here to declare subsystem dependencies.
    roboDT = robotDT;
    distToTravel_in = inchesToTravel;
     // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboDT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roboDT.resetEncoders();
    
    // Reset the variables so our desired change in position is the same everytime and not (desiredPosition) * numberOfCycles
    initialLeftPosition = (int)roboDT.getLeftPosition();
    initialRightPosition = (int)roboDT.getRightPosition();
    desiredLeftPosition = 0;
    desiredRightPosition = 0;
    initialLeftPosition = 0;
    initialRightPosition = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roboDT.arcadeDrive(0, -0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roboDT.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Left encoder counts down so that's why we're doing subtraction 
    desiredLeftPosition = initialLeftPosition - roboDT.inchesToCounts(distToTravel_in, 
    Constants.GenConstants.REV_ENCODER_CPR);
    currentLeftPosition = (int)roboDT.getLeftPosition();
   
    // Right counts up so you do the addition 
    desiredRightPosition = roboDT.inchesToCounts(distToTravel_in, 
    Constants.GenConstants.REV_ENCODER_CPR) + initialRightPosition;
    currentRightPosition = (int)roboDT.getRightPosition();
    
    SmartDashboard.putNumber("Desired Left", desiredLeftPosition);
    SmartDashboard.putNumber("Desired Right", desiredRightPosition);
    SmartDashboard.putNumber("Current Left", currentLeftPosition);
    SmartDashboard.putNumber("Current Right", currentRightPosition);
    SmartDashboard.putNumber("Initial Left", initialLeftPosition);
    SmartDashboard.putNumber("Initial Right", initialRightPosition);

    if ((currentRightPosition >= desiredRightPosition) && (currentLeftPosition<=desiredLeftPosition)){
      return true;
  } else {
    return false;
  }
   
  }
}
