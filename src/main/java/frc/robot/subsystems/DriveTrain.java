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
import com.revrobotics.CANPIDController.AccelStrategy;
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
  private static final double mMaxRPM = 5676 / 12.05; //REV Neo free sped = 5676 rpm, gearbox = 12.05:1;

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

    //driveTrain = new DifferentialDrive(leftDriveMotor1,rightDriveMotor1);

    mLeftDrivePID.setP(DriveConstants.kP);
    mLeftDrivePID.setI(DriveConstants.kI);
    mLeftDrivePID.setD(DriveConstants.kD);
    //leftDrivePID.setIZone(kIz);
    //leftDrivePID.setFF(kFF);
    mLeftDrivePID.setOutputRange(-1, 1);
    mLeftDrivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    mRightDrivePID.setP(DriveConstants.kP);
    mRightDrivePID.setI(DriveConstants.kI);
    mRightDrivePID.setD(DriveConstants.kD);
    //rightDrivePID.setIZone(kIz);
    //rightDrivePID.setFF(kFF);
    mRightDrivePID.setOutputRange(-1, 1);
    mRightDrivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 1);

    mRightDrivePID.setSmartMotionMaxAccel(20, 1);
    mRightDrivePID.setSmartMotionAllowedClosedLoopError(1, 1);
    mLeftDrivePID.setSmartMotionMaxAccel(20, 0);
    mLeftDrivePID.setSmartMotionAllowedClosedLoopError(1, 0);
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

    mLeftDrivePID.setReference((y+x)*mMaxRPM, ControlType.kVelocity);
    mRightDrivePID.setReference(-(y-x)*mMaxRPM, ControlType.kVelocity);

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

  public void resetEncoders() {
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
  }

  public void updateSmartdashboard(){
    SmartDashboard.putNumber("Left Encoder Position", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Position", mRightEncoder.getPosition());
  }
}

