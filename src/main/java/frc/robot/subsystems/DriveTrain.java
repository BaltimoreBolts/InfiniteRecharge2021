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
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

import java.sql.Time;
import java.util.Map;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

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

  private DifferentialDrive driveTrain;
  private AHRS mNavx = null; // usb port coms w/ navx
  private final DifferentialDriveOdometry mOdometry;
  private ShuffleboardTab mainTab; 
  private double mLSetpoint = 0;
  private double mRSetpoint = 0;
  private double dt = 0.02; // wpilib runs at 50Hz
      
  
  /**
   * Creates a new DriveTrain.
   */

/* 
** PURPOSE: DriveTrain subsystem
** STATUS: Tested pretty well
*/
  public DriveTrain() {
    // Initialize all of the drive motors and set to correct settings. Burn to flash.
    
    setupMotors();
    mainTab = Shuffleboard.getTab("Main");
    try {
      mNavx = new AHRS(AutoConstants.NAVX_PORT, (byte) 100);
    } catch (RuntimeException ex){
      DriverStation.reportError("Error instantiantiating navX MXP: " + ex.getMessage(), true);
      SmartDashboard.putBoolean("[Drivetrain] USB navX Error", true);
    }
    mNavx.calibrate();
    mNavx.reset();
    mNavx.zeroYaw();
    resetEncoders();
    mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(mNavx.getAngle()));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mOdometry.update(
      Rotation2d.fromDegrees(mNavx.getAngle()), mLeftEncoder.getPosition() * Constants.in2m(DriveConstants.WHEEL_CIRCUMFERENCE), 
      mRightEncoder.getPosition() * Constants.in2m(DriveConstants.WHEEL_CIRCUMFERENCE)
    );
    updateSmartdashboard();
    // updateShuffleboard();

  }

  public void motionProfileDriving(double x, double y){
    double left_set_point = (y+x)*DriveConstants.MAX_RPM;
    double right_set_point = -(y-x)*DriveConstants.MAX_RPM;

    double leftError = left_set_point - mLSetpoint;
    if (leftError != 0) {
      mLSetpoint = (leftError)/Math.abs(leftError) * DriveConstants.MAX_ACC * dt + mLSetpoint;
    }
    double rightError = right_set_point - mRSetpoint;
    if (rightError != 0) {
      mRSetpoint = (rightError)/Math.abs(rightError) * DriveConstants.MAX_ACC * dt + mRSetpoint;
    }
    
    double leftDesiredAccel = Math.abs(leftError/dt);
    double rightDesiredAccel = Math.abs(rightError/dt);
    mLSetpoint = (leftDesiredAccel < DriveConstants.MAX_ACC) ? left_set_point : mLSetpoint;
    mRSetpoint = (rightDesiredAccel < DriveConstants.MAX_ACC) ? right_set_point : mRSetpoint;

    SmartDashboard.putNumber("[Drivertrain] Left Setpoint", mLSetpoint); 
    SmartDashboard.putNumber("[Drivertrain] Right Setpoint", mRSetpoint);

    mLeftDrivePID.setReference(mLSetpoint, ControlType.kVelocity,1);
    mRightDrivePID.setReference(mRSetpoint, ControlType.kVelocity,1);
  }

  public Pose2d getPose(){
    return mOdometry.getPoseMeters();
  }

  private Double mps2rpm(Double mps){
    return DriveConstants.GEARBOX_RATIO * 60.0 * Constants.m2in(mps) / DriveConstants.WHEEL_CIRCUMFERENCE;
  }

  public void setWheelSpeeds(Double left, Double right){
    right = mps2rpm(right);
    left = mps2rpm(left);
    mRightDrivePID.setReference(-right,ControlType.kVelocity, 1);
    mLeftDrivePID.setReference(left, ControlType.kVelocity, 1);
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    mOdometry.resetPosition(pose, Rotation2d.fromDegrees(mNavx.getAngle()));
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    double left_distance = mLeftEncoder.getPosition() * Constants.in2m(DriveConstants.WHEEL_CIRCUMFERENCE);
    double right_distance = mRightEncoder.getPosition() * Constants.in2m(DriveConstants.WHEEL_CIRCUMFERENCE);

    return (left_distance + right_distance) / 2.0;
  }

  /**
   * Zeroes the heading of the robot.
  */
  public void zeroHeading(){
    mNavx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
  */
  public double getHeading(){
    return mNavx.getAngle(); 
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
  */
  public double getTurnRate(){
    return -mNavx.getRate(); 
  }

  public void stopDT(){
    mLeftDriveMotor1.set(0);
    mRightDriveMotor1.set(0);
  }

  public void arcadeDrive(double x, double y) {
		driveTrain.arcadeDrive(y, x); // Moving the stick forward (+y) moves the robot forward, left/right on the stick makes the robot spin
  }

  public void closedLoopArcadeDrive(double x, double y){

    x = Math.pow(x,3); 
    // y = Math.pow(y,3);

    double left_set_point = (y+x)*DriveConstants.MAX_RPM;
    double right_set_point = -(y-x)*DriveConstants.MAX_RPM;

    // SmartDashboard.putNumber("[Drivetrain] Left set point", left_set_point);

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
  
  public void resetEncoders() {
    mRightBuiltInEncoder.setPosition(0);
    mLeftBuiltInEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
    mLeftEncoder.setPosition(0);
  }

  private void setupMotors(){
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

    mLeftEncoder = mLeftDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    mLeftEncoder.setInverted(true);
    mLeftBuiltInEncoder = mLeftDriveMotor2.getEncoder();
    mRightEncoder = mRightDriveMotor2.getAlternateEncoder(kAltEncType,GenConstants.REV_ENCODER_CPR);
    mRightBuiltInEncoder = mRightDriveMotor2.getEncoder();

    mLeftDriveMotor1.burnFlash();
    mLeftDriveMotor2.burnFlash();
    mRightDriveMotor1.burnFlash();
    mRightDriveMotor2.burnFlash();

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
  }

  private void updateSmartdashboard(){
    SmartDashboard.putNumber("[Drivetrain] Left Encoder Position", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("[Drivetrain] Right Encoder Position", mRightEncoder.getPosition());
    SmartDashboard.putNumber("[Drivetrain] Left Encoder Velocity", mLeftEncoder.getVelocity());
    SmartDashboard.putNumber("[Drivetrain] Right Encoder Velocity", mRightEncoder.getVelocity());
    SmartDashboard.putNumber("[Drivetrain] Heading", mOdometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("[Drivetrain] NavX Angle", mNavx.getAngle());
    SmartDashboard.putBoolean("[Drivetrain] NavX Calibration", mNavx.isCalibrating());
    SmartDashboard.putBoolean("[Drivetrain] NavX Connected", mNavx.isConnected());
    SmartDashboard.putNumber("[Drivetrain] Odometry X Position", mOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("[Drivetrain] Odometry Y Position", mOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("[Drivetrain] NavX X Velocity", mNavx.getVelocityX());
    SmartDashboard.putNumber("[Drivetrain] NavX Y Velocity", mNavx.getVelocityY());
  }
  
  // private void updateShuffleboard(){
  //   ShuffleboardLayout drivetrainData = mainTab.getLayout("Drivetrain", BuiltInLayouts.kGrid).withProperties(Map.of("Number of columns", 2, "Number of rows", 3));
  //   // drivetrainData.add("Left Encoder", mLeftEncoder).withWidget(BuiltInWidgets.kEncoder);
  //   drivetrainData.add("NavX Heading", mNavx).withWidget(BuiltInWidgets.kGyro);
  //   drivetrainData.add("Odometry Heading", mOdometry.getPoseMeters().getRotation().getDegrees()).withWidget(BuiltInWidgets.kDial);
  // }

  public AHRS getNavx(){
    return mNavx;
  }
}

