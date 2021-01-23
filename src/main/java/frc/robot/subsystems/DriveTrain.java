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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftDriveMotor1;
  private CANSparkMax leftDriveMotor2;
  private CANSparkMax rightDriveMotor1;
  private CANSparkMax rightDriveMotor2;

  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;

  private DifferentialDrive driveTrain;
  
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
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
    leftDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightDriveMotor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightDriveMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftDriveMotor2.follow(leftDriveMotor1);
    rightDriveMotor2.follow(rightDriveMotor1);
    leftDriveMotor1.burnFlash();
    leftDriveMotor2.burnFlash();
    rightDriveMotor1.burnFlash();
    rightDriveMotor2.burnFlash();

    leftEncoder = leftDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    rightEncoder = rightDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);

    driveTrain = new DifferentialDrive(leftDriveMotor1,rightDriveMotor1);
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

