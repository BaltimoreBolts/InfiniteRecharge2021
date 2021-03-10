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

import java.util.function.BiConsumer;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.ml.Ml;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax mLeftDriveMotor1;
  private CANSparkMax mLeftDriveMotor2;
  private CANSparkMax mRightDriveMotor1;
  private CANSparkMax mRightDriveMotor2;
  private CANPIDController mLeftDrivePID;
  private CANPIDController mRightDrivePID;
  private CANEncoder mLeftEncoder;
  private CANEncoder mLeftBuiltInEncoder;
  private CANEncoder mRightEncoder;
  private CANEncoder mRightBuiltInEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;

  private double mTrajLeftSpeed = 0;
  private double mTrajRightSpeed = 0;

  private DifferentialDrive driveTrain;
  private final Gyro mGyro = new ADXRS450_Gyro(); // TODO add gyro, im not sure what type we have
  private final DifferentialDriveOdometry mOdometry;
      
  public BiConsumer<Double, Double> wheelSpeeds = (x,y) -> {
    mTrajLeftSpeed = x;
    mTrajRightSpeed = y;
  };
  
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

    mLeftDriveMotor1.follow(mLeftDriveMotor2);
    mRightDriveMotor1.follow(mRightDriveMotor2);

    mLeftDriveMotor1.burnFlash();
    mLeftDriveMotor2.burnFlash();
    mRightDriveMotor1.burnFlash();
    mRightDriveMotor2.burnFlash();

    mLeftEncoder = mLeftDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    mLeftBuiltInEncoder = mLeftDriveMotor2.getEncoder();
    mLeftEncoder.setInverted(true);
    mRightEncoder = mRightDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    mRightBuiltInEncoder = mRightDriveMotor2.getEncoder();

    mLeftDrivePID = mLeftDriveMotor2.getPIDController();
    mLeftDrivePID.setFeedbackDevice(mLeftBuiltInEncoder);

    mRightDrivePID = mRightDriveMotor2.getPIDController();
    mRightDrivePID.setFeedbackDevice(mRightBuiltInEncoder);

    // there are two different PID slots per side, one for moving forward (0) and one for autonomous turning (1)
    mLeftDrivePID.setP(DriveConstants.kP);
    mLeftDrivePID.setI(DriveConstants.kI);
    mLeftDrivePID.setD(DriveConstants.kD);
    //leftDrivePID.setIZone(kIz);
    mLeftDrivePID.setFF(DriveConstants.kFF);
    mLeftDrivePID.setOutputRange(-1, 1);

    mRightDrivePID.setP(DriveConstants.kP);
    mRightDrivePID.setI(DriveConstants.kI);
    mRightDrivePID.setD(DriveConstants.kD);
    //rightDrivePID.setIZone(kIz);
    mRightDrivePID.setFF(DriveConstants.kFF);
    mRightDrivePID.setOutputRange(-1, 1);


    mLeftDrivePID.setP(DriveConstants.kP,1);
    mLeftDrivePID.setI(DriveConstants.kI,1);
    mLeftDrivePID.setD(DriveConstants.kD,1);
    mLeftDrivePID.setFF(DriveConstants.kFF,1);

    mRightDrivePID.setP(DriveConstants.kP,1);
    mRightDrivePID.setI(DriveConstants.kI,1);
    mRightDrivePID.setD(DriveConstants.kD,1);
    mRightDrivePID.setFF(DriveConstants.kFF,1);

    
    mLeftDrivePID.setP(DriveConstants.kP,2);
    mLeftDrivePID.setI(DriveConstants.kI,2);
    mLeftDrivePID.setD(DriveConstants.kD,2);
    mLeftDrivePID.setFF(DriveConstants.kFF,2);

    mRightDrivePID.setP(DriveConstants.kP,2);
    mRightDrivePID.setI(DriveConstants.kI,2);
    mRightDrivePID.setD(DriveConstants.kD,2);
    mRightDrivePID.setFF(DriveConstants.kFF,2);


    mRightDrivePID.setSmartMotionMaxAccel(DriveConstants.MAX_ACC, 1);
    mRightDrivePID.setSmartMotionMaxVelocity(DriveConstants.MAX_VEL, 1);
    mRightDrivePID.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, 1);
    mRightDrivePID.setSmartMotionAllowedClosedLoopError(DriveConstants.ALLOWED_ERROR, 1);

    mRightDrivePID.setSmartMotionMaxAccel(DriveConstants.MAX_ACC/3.0, 2);
    mRightDrivePID.setSmartMotionMaxVelocity(DriveConstants.MAX_VEL/3.0, 2);
    mRightDrivePID.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, 2);
    mRightDrivePID.setSmartMotionAllowedClosedLoopError(DriveConstants.ALLOWED_ERROR, 2);

    mLeftDrivePID.setSmartMotionMaxAccel(DriveConstants.MAX_ACC, 1);
    mLeftDrivePID.setSmartMotionMaxVelocity(DriveConstants.MAX_VEL, 1);
    mLeftDrivePID.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, 1);
    mLeftDrivePID.setSmartMotionAllowedClosedLoopError(DriveConstants.ALLOWED_ERROR, 1);

    mLeftDrivePID.setSmartMotionMaxAccel(DriveConstants.MAX_ACC/3.0, 2);
    mLeftDrivePID.setSmartMotionMaxVelocity(DriveConstants.MAX_VEL/3.0, 2);
    mLeftDrivePID.setSmartMotionAccelStrategy(CANPIDController.AccelStrategy.kTrapezoidal, 2);
    mLeftDrivePID.setSmartMotionAllowedClosedLoopError(DriveConstants.ALLOWED_ERROR, 2);

    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mOdometry.update(
      mGyro.getRotation2d(), mLeftEncoder.getPosition() * DriveConstants.WHEEL_CIRCUMFERENCE * GenConstants.IN_TO_M, 
      mRightEncoder.getPosition() * DriveConstants.WHEEL_CIRCUMFERENCE * GenConstants.IN_TO_M
    );
    updateSmartdashboard();
  }

  public Pose2d getPose(){
    return mOdometry.getPoseMeters();
  }

  public void setWheelSpeeds(Double left, Double right){
    mTrajRightSpeed = right;
    mTrajLeftSpeed = left;
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    mOdometry.resetPosition(pose, mGyro.getRotation2d());
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    double left_distance = mLeftEncoder.getPosition() * DriveConstants.WHEEL_CIRCUMFERENCE * GenConstants.IN_TO_M;
    double right_distance = mRightEncoder.getPosition() * DriveConstants.WHEEL_CIRCUMFERENCE * GenConstants.IN_TO_M;

    return (left_distance + right_distance) / 2.0;
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading(){
    mGyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading(){
    return mGyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate(){
    return -mGyro.getRate();
  }

  public void stopDT(){
    mLeftDriveMotor1.set(0);
    mRightDriveMotor1.set(0);
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

    double left_set_point = (y+x)*DriveConstants.MAX_RPM;
    double right_set_point = -(y-x)*DriveConstants.MAX_RPM;

    SmartDashboard.putNumber("[Drivetrain] left set point", left_set_point);

    mLeftDrivePID.setReference(left_set_point, ControlType.kSmartVelocity,1);
    mRightDrivePID.setReference(right_set_point, ControlType.kSmartVelocity,1);

  }

  public double getLeftPosition() {
    return mLeftBuiltInEncoder.getPosition();
  }

  public double getRightPosition() {
    return mRightBuiltInEncoder.getPosition();
  }
  public double inchesToCounts(double inches, int CPR ){
    double Counts = 0;
    // Due to the diameter of the wheels being 8, we divided by 8 PI which is the circumfrence/
    //Counts = (int)Math.ceil(CPR * inches/(8* Math.PI));
    Counts = inches / (3 * Math.PI);
    return Counts;

  }

  public void driveDistance(double inches){
    double rotations = DriveConstants.GEARBOX_RATIO * inches / DriveConstants.WHEEL_CIRCUMFERENCE;
    mLeftDrivePID.setReference(rotations, ControlType.kSmartMotion, 2);
    mRightDrivePID.setReference(-rotations, ControlType.kSmartMotion, 2);
    // double distanceTraveled = DriveConstants.WHEEL_CIRCUMFERENCE * (mLeftEncoder.getPosition() + mRightEncoder.getPosition()) / 2.0;
    // return (distanceTraveled >= inches);
  }

  public boolean autonTurn(double inches, double degrees, boolean clockwise){

    double leftWheelTurns = 0;
    double rightWheelTurns = 0;

    if (clockwise) { // calculate inner and outer turning circumference
      leftWheelTurns = ((inches + (DriveConstants.TRACK / 2.0)) * Math.PI * 2.0) * degrees/360.0;
      rightWheelTurns = ((inches - (DriveConstants.TRACK / 2.0)) * Math.PI * 2.0) * degrees/360.0;
    } else {
      leftWheelTurns = ((inches - (DriveConstants.TRACK / 2.0)) * Math.PI * 2.0) * degrees/360.0;
      rightWheelTurns = ((inches + (DriveConstants.TRACK / 2.0)) * Math.PI * 2.0) * degrees/360.0;
    }
    
    mLeftDrivePID.setReference(leftWheelTurns, ControlType.kSmartMotion, 2);
    mRightDrivePID.setReference(-rightWheelTurns, ControlType.kSmartMotion, 2);

    double leftDistTraveled  =  mLeftEncoder.getPosition();
    double rightDistTraveled =  mRightEncoder.getPosition();

    return ((leftDistTraveled >= leftWheelTurns) && (rightDistTraveled >= rightWheelTurns));
  }

  public void resetEncoders() {
    mRightBuiltInEncoder.setPosition(0);
    mLeftBuiltInEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
  }

  public void updateSmartdashboard(){
    SmartDashboard.putNumber("[Drivetrain] Left Encoder Position", mLeftBuiltInEncoder.getPosition());
    SmartDashboard.putNumber("[Drivetrain] Right Encoder Position", mRightBuiltInEncoder.getPosition());
    SmartDashboard.putNumber("[Drivetrain] Left Encoder Velocity", mLeftBuiltInEncoder.getVelocity());
    SmartDashboard.putNumber("[Drivetrain] Right Encoder Velocity", mLeftBuiltInEncoder.getVelocity());
    
  }
}

