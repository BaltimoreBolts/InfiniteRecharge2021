/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GenConstants;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax mLeftDriveMotor1;
  private CANSparkMax mLeftDriveMotor2;
  private CANSparkMax mRightDriveMotor1;
  private CANSparkMax mRightDriveMotor2;
  private CANPIDController mLeftDrivePID;
  private CANPIDController mRightDrivePID;
  private CANEncoder mLeftEncoder;
  private CANEncoder mRightEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;

  private DifferentialDrive driveTrain;
  
  /**
   * Creates a new DriveTrain.
   */

/* 
** PURPOSE: DriveTrain subsystem
** STATUS: Tested pretty well
*/
  public DriveTrain() {

    // Initialize all of the drive motors and set to correct settings. Burn to flash.
    mLeftDriveMotor1 = new CANSparkMax(DriveConstants.LEFT_DRIVE_MOTOR1, MotorType.kBrushless);
    mLeftDriveMotor2 = new CANSparkMax(DriveConstants.LEFT_DRIVE_MOTOR2, MotorType.kBrushless);
    mRightDriveMotor1 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_MOTOR1, MotorType.kBrushless);
    mRightDriveMotor2 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_MOTOR2, MotorType.kBrushless);
    mLeftDriveMotor1.restoreFactoryDefaults();
    mLeftDriveMotor2.restoreFactoryDefaults();
    mRightDriveMotor1.restoreFactoryDefaults();
    mRightDriveMotor2.restoreFactoryDefaults();
    mLeftDriveMotor1.setSmartCurrentLimit(40);
    mLeftDriveMotor2.setSmartCurrentLimit(40);
    mRightDriveMotor1.setSmartCurrentLimit(40);
    mRightDriveMotor2.setSmartCurrentLimit(40);
    mLeftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mLeftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mLeftDriveMotor2.follow(mLeftDriveMotor1);
    mRightDriveMotor2.follow(mRightDriveMotor1);

    mLeftDriveMotor1.burnFlash();
    mLeftDriveMotor2.burnFlash();
    mRightDriveMotor1.burnFlash();
    mRightDriveMotor2.burnFlash();

    mLeftEncoder = mLeftDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    mLeftEncoder.setInverted(true);
    mRightEncoder = mRightDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    
    mLeftDrivePID = mLeftDriveMotor1.getPIDController();
    mLeftDrivePID.setFeedbackDevice(mLeftEncoder);

    mRightDrivePID = mRightDriveMotor1.getPIDController();
    mRightDrivePID.setFeedbackDevice(mRightEncoder);


    // there are two different PID slots per side, one for moving forward (0) and one for autonomous turning (1)
    mLeftDrivePID.setP(DriveConstants.kP, 0);
    mLeftDrivePID.setI(DriveConstants.kI, 0);
    mLeftDrivePID.setD(DriveConstants.kD, 0);
    //leftDrivePID.setIZone(kIz);
    //leftDrivePID.setFF(kFF);
    mLeftDrivePID.setP(DriveConstants.kP, 1);
    mLeftDrivePID.setI(DriveConstants.kI, 1);
    mLeftDrivePID.setD(DriveConstants.kD, 1);
    mLeftDrivePID.setOutputRange(-1, 1, 0);
    mLeftDrivePID.setOutputRange(-1, 1, 1);

    mRightDrivePID.setP(DriveConstants.kP, 0);
    mRightDrivePID.setI(DriveConstants.kI, 0);
    mRightDrivePID.setD(DriveConstants.kD, 0);
    mRightDrivePID.setP(DriveConstants.kP, 1);
    mRightDrivePID.setI(DriveConstants.kI, 1);
    mRightDrivePID.setD(DriveConstants.kD, 1);
    //rightDrivePID.setIZone(kIz);
    //rightDrivePID.setFF(kFF);
    mRightDrivePID.setOutputRange(-1, 1, 0);
    mRightDrivePID.setOutputRange(-1, 1, 1);

    mRightDrivePID.setSmartMotionMaxAccel(DriveConstants.MAX_ACC, 0);
    mRightDrivePID.setSmartMotionMaxVelocity(DriveConstants.MAX_VEL, 0);

    mRightDrivePID.setSmartMotionMaxAccel(DriveConstants.MAX_ACC/2, 1);
    mRightDrivePID.setSmartMotionMaxVelocity(DriveConstants.MAX_VEL/2, 1);

    mRightDrivePID.setSmartMotionAllowedClosedLoopError(DriveConstants.ALLOWED_ERROR, 0);
    mRightDrivePID.setSmartMotionAllowedClosedLoopError(DriveConstants.ALLOWED_ERROR, 1);

    mLeftDrivePID.setSmartMotionMaxAccel(DriveConstants.MAX_ACC, 0);
    mLeftDrivePID.setSmartMotionMaxVelocity(DriveConstants.MAX_VEL, 0);

    mLeftDrivePID.setSmartMotionMaxAccel(DriveConstants.MAX_ACC/2, 1);
    mLeftDrivePID.setSmartMotionMaxVelocity(DriveConstants.MAX_VEL/2, 1);


    mLeftDrivePID.setSmartMotionAllowedClosedLoopError(DriveConstants.ALLOWED_ERROR, 0);
    mLeftDrivePID.setSmartMotionAllowedClosedLoopError(DriveConstants.ALLOWED_ERROR, 1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartdashboard();
  }
  
  public void arcadeDrive(double x, double y) {
		driveTrain.arcadeDrive(y, x); // Moving the stick forward (+y) moves the robot forward, left/right on the stick makes the robot spin
  }

  public void closedLoopArcadeDrive(double x, double y){
    /*if (Math.abs(x) < 0.05){
      x = 0;
    }
    if (Math.abs(y) < 0.05){
      y = 0;
    }
    */
    x = Math.pow(x,3); 
    y = Math.pow(y,3);

    mLeftDrivePID.setReference((y+x)*DriveConstants.MAX_RPM, ControlType.kVelocity);
    mRightDrivePID.setReference(-(y-x)*DriveConstants.MAX_RPM, ControlType.kVelocity);

  }

  public double getLeftPosition() {
    return mLeftEncoder.getPosition();
  }

  public double getRightPosition() {
    return mRightEncoder.getPosition();
  }
  public double inchesToCounts(double inches, int CPR ){
    double Counts = 0;
    // Due to the diameter of the wheels being 8, we divided by 8 PI which is the circumfrence/
    //Counts = (int)Math.ceil(CPR * inches/(8* Math.PI));
    Counts = inches / (3 * Math.PI);
    return Counts;

  }

  public boolean driveDistance(double inches){
    double rotations = inches / DriveConstants.WHEEL_CIRCUMFERENCE;
    mLeftDrivePID.setReference(rotations, ControlType.kSmartMotion, 0);
    mRightDrivePID.setReference(rotations, ControlType.kSmartMotion, 0);
    double distanceTraveled = DriveConstants.WHEEL_CIRCUMFERENCE * (mLeftEncoder.getPosition() + mRightEncoder.getPosition()) / 2.0;
    return (distanceTraveled >= inches);
  }

  public boolean autonTurn(double inches, double degrees, boolean clockwise){

    double leftWheelTurns = 0;
    double rightWheelTurns = 0;

    if (clockwise) { // calculate inner and outer turning circumference
      leftWheelTurns = ((inches + (DriveConstants.TRACK / 2.0)) * Math.PI) * degrees/360.0;
      rightWheelTurns = ((inches - (DriveConstants.TRACK / 2.0)) * Math.PI) * degrees/360.0;
    } else {
      leftWheelTurns = ((inches - (DriveConstants.TRACK / 2.0)) * Math.PI) * degrees/360.0;
      rightWheelTurns = ((inches + (DriveConstants.TRACK / 2.0)) * Math.PI) * degrees/360.0;
    }
    
    mLeftDrivePID.setReference(leftWheelTurns, ControlType.kSmartMotion, 1);
    mRightDrivePID.setReference(rightWheelTurns, ControlType.kSmartMotion, 1);

    double leftDistTraveled =  mLeftEncoder.getPosition();
    double rightDistTraveled =  mRightEncoder.getPosition();

    return ((leftDistTraveled >= leftWheelTurns) && (rightDistTraveled >= rightWheelTurns));
  }

  public void resetEncoders() {
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
  }

  public void updateSmartdashboard(){
    SmartDashboard.putNumber("Left Encoder Position", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Position", mRightEncoder.getPosition());
  }
}

