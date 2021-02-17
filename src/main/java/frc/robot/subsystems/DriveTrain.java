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
  private CANSparkMax leftDriveMotor1;
  private CANSparkMax leftDriveMotor2;
  private CANSparkMax rightDriveMotor1;
  private CANSparkMax rightDriveMotor2;
  private CANPIDController leftDrivePID;
  private CANPIDController rightDrivePID;
  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private static final double maxRPM = 5676 / 12.05; //REV Neo free sped = 5676 rpm, gearbox = 12.05:1;
  private double kP = 0.005; // 2e-5 initial test value 
  private double kI = 0; // 0 initial test value 
  private double kD = 0; // 0 initial test value 
  private double kFF = 0; //0.000165; // 0.000165 initial test value

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
    leftDriveMotor1 = new CANSparkMax(DriveConstants.LEFT_DRIVE_MOTOR1, MotorType.kBrushless);
    leftDriveMotor2 = new CANSparkMax(DriveConstants.LEFT_DRIVE_MOTOR2, MotorType.kBrushless);
    rightDriveMotor1 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_MOTOR1, MotorType.kBrushless);
    rightDriveMotor2 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_MOTOR2, MotorType.kBrushless);
    leftDriveMotor1.restoreFactoryDefaults();
    leftDriveMotor2.restoreFactoryDefaults();
    rightDriveMotor1.restoreFactoryDefaults();
    rightDriveMotor2.restoreFactoryDefaults();
    leftDriveMotor1.setSmartCurrentLimit(40);
    leftDriveMotor2.setSmartCurrentLimit(40);
    rightDriveMotor1.setSmartCurrentLimit(40);
    rightDriveMotor2.setSmartCurrentLimit(40);
    leftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    leftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);

    leftDriveMotor2.follow(leftDriveMotor1);
    rightDriveMotor2.follow(rightDriveMotor1);

    leftDriveMotor1.burnFlash();
    leftDriveMotor2.burnFlash();
    rightDriveMotor1.burnFlash();
    rightDriveMotor2.burnFlash();

    leftEncoder = leftDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    leftEncoder.setInverted(true);
    rightEncoder = rightDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    
    leftDrivePID = leftDriveMotor1.getPIDController();
    leftDrivePID.setFeedbackDevice(leftEncoder);

    rightDrivePID = rightDriveMotor1.getPIDController();
    rightDrivePID.setFeedbackDevice(rightEncoder);

    //driveTrain = new DifferentialDrive(leftDriveMotor1,rightDriveMotor1);

    leftDrivePID.setP(kP);
    leftDrivePID.setI(kI);
    leftDrivePID.setD(kD);
    //leftDrivePID.setIZone(kIz);
    //leftDrivePID.setFF(kFF);
    leftDrivePID.setOutputRange(-1, 1);
    leftDrivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    rightDrivePID.setP(kP);
    rightDrivePID.setI(kI);
    rightDrivePID.setD(kD);
    //rightDrivePID.setIZone(kIz);
    //rightDrivePID.setFF(kFF);
    rightDrivePID.setOutputRange(-1, 1);
    rightDrivePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());
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

    leftDrivePID.setReference((y+x)*maxRPM, ControlType.kVelocity);
    rightDrivePID.setReference(-(y-x)*maxRPM, ControlType.kVelocity);

  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }
  public double inchesToCounts(double inches, int CPR ){
    double Counts = 0;
    // Due to the diameter of the wheels being 8, we divided by 8 PI which is the circumfrence/
    //Counts = (int)Math.ceil(CPR * inches/(8* Math.PI));
    Counts = inches / (3 * Math.PI);
    return Counts;

  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

}

