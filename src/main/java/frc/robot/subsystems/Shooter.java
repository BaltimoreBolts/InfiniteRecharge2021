/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import java.lang.Math;
import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GenConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterControlState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/* 
** PURPOSE: Shooter subsystem 
** STATUS: Motor functions are good. PID is not. 
** I believe network table connection was working (we were getting values from the pi), 
** but not sure if things have changed since chameleon vision changed
*/
public class Shooter extends SubsystemBase {
  private CANSparkMax leftShooterMotor;
  private CANSparkMax rightShooterMotor;
  private double desiredRPM = 0;
  private double absoluteRPMError = 0;
  private double motor1ShooterSpeed = 0;
  private double leftShooterRPM = 0.0;
  private double leftShooterVoltage = 0.0;
  private double leftShooterDutyCycle = 0.0;

  private CANPIDController shooterPID;
  private double kP = 0.001; 
  private double kI = 0; 
  private double kD = 0; 
  private double kFF = 0.000013;
  private int kFFSampleCount = 0;
  private int kFFCircularBufferCounter = 0;
  private double[] kFFCircularBuffer = new double[ShooterConstants.kFFCircularBufferSize];
  private boolean onTarget = false;
  private CANEncoder shooterEncoder;
  
  // Network table for chameleon vision
  NetworkTableInstance table = NetworkTableInstance.getDefault();
  NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("PsThreeCam");
  private NetworkTableEntry targetPose;
  private double targetArr[] = {0,0,0};
  private double x, y, angle = 0;
  private double fudgeFactor = 0; 
  private boolean readyToFire = false;
  private ShooterControlState shooterControlState = ShooterControlState.IDLE;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // Set up motors
    leftShooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_CHIP, MotorType.kBrushed);
    rightShooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DALE, MotorType.kBrushed);
    leftShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.setSmartCurrentLimit(30);
    rightShooterMotor.setSmartCurrentLimit(30);
    leftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    leftShooterMotor.burnFlash();
    rightShooterMotor.burnFlash();
    // Set Right to follow Left, but inverted
    rightShooterMotor.follow(leftShooterMotor,true);
    
    // Instantiate encoder
    shooterEncoder = leftShooterMotor.getEncoder(EncoderType.kQuadrature, GenConstants.REV_ENCODER_CPR);

    // Start PID
    shooterPID = leftShooterMotor.getPIDController();
    shooterPID.setFeedbackDevice(shooterEncoder);
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(-1,1);
    shooterPID.setIZone(1000);

    // Prints the initial PID values to smart dashboard
    SmartDashboard.putNumber("Current pVal = ", kP);
    SmartDashboard.putNumber("Current iVal = ", kI);
    SmartDashboard.putNumber("Current dVal = ", kD);
    SmartDashboard.putNumber("Current ffVal = ", kFF);
    SmartDashboard.putNumber("Shooter Speed = ", motor1ShooterSpeed);
    SmartDashboard.putNumber("Desired RPM = ", desiredRPM);
    SmartDashboard.putBoolean("Value or SetPID:", true); // Set to true for using "Shooter Motor Speed" to control shooter speed
    SmartDashboard.putNumber("Calculated velocity", 0);
    SmartDashboard.putNumber("Calculated RPM", 0);
    SmartDashboard.putNumber("Camera X Dist", x);
    SmartDashboard.putNumber("Camera Y Dist", y);
    SmartDashboard.putNumber("Camera Angle Dist", angle);
    SmartDashboard.putNumber("Shooter Fudge Factor", fudgeFactor);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder", shooterEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Vel", shooterEncoder.getVelocity());

    // Get distance data from camera 
    targetPose = cameraTable.getEntry("targetPose");
    targetArr = targetPose.getDoubleArray(targetArr);

    // Save the most recent data
    x = targetArr[0];
    y = targetArr[1];
    angle = targetArr[2];

    // Display on SmartDashboard
    SmartDashboard.putNumber("Camera X Dist", x);
    SmartDashboard.putNumber("Camera Y Dist", y);
    SmartDashboard.putNumber("Camera Angle Dist", angle);
    SmartDashboard.getNumber("Shooter Fudge Factor", fudgeFactor);
    SmartDashboard.putString("Shooter state", shooterControlState.toString());
    
    // PIDTuner(); // Comment this out once we figure out our PID values.
    // getNeededRPM();
    closedLoopStateMachineManager();
  }

  public void closedLoopStateMachineManager(){
    leftShooterRPM = shooterEncoder.getVelocity();
    leftShooterVoltage = leftShooterMotor.getBusVoltage();    
    leftShooterDutyCycle = leftShooterMotor.getAppliedOutput();

    // State Machine
    if (shooterControlState == ShooterControlState.IDLE){

      kP = 0.0005; 
      kI = 0.0000025; 
      kD = 0; 
      kFF = 0.000013;
      shooterPID.setP(kP);
      shooterPID.setI(kI);
      shooterPID.setD(kD);
      shooterPID.setFF(kFF);
      leftShooterMotor.set(0);

    } else if (shooterControlState == ShooterControlState.SPINUP) {
      // figure out desired speed
      //desiredRPM = getNeededRPM();
      desiredRPM = -8000;
      // pid spin up
      SetShooterSpeed(desiredRPM);
      
      shooterControlState = ShooterControlState.HOLDWHENREADY;

    } else if (shooterControlState == ShooterControlState.HOLDWHENREADY) {
      SmartDashboard.putNumber("Shooter kF", kFcalculator(leftShooterDutyCycle, leftShooterRPM));
      absoluteRPMError = Math.abs(desiredRPM - leftShooterRPM);

      if (absoluteRPMError < 100 && !onTarget) {
        onTarget = true;
      } else if (absoluteRPMError > 100) {
        resetHold(); // onTarget is now false
      }

      if (onTarget) {
        // record value in kFFCircularBuffer
        kFFCircularBuffer[kFFCircularBufferCounter % ShooterConstants.kFFCircularBufferSize] = kFcalculator(leftShooterVoltage, leftShooterRPM);
        kFFCircularBufferCounter++;
        kFFSampleCount++;
      }

      if (kFFSampleCount >= ShooterConstants.kFFCircularBufferSize) {
        shooterControlState = ShooterControlState.HOLD;
      } else {
        SetShooterSpeed(desiredRPM);
      }

      double kFFToAddToBuffer = kFcalculator(leftShooterMotor.getBusVoltage(), shooterEncoder.getVelocity()); // TODO not sure about units of get velocity
      kFFCircularBuffer[kFFCircularBufferCounter % ShooterConstants.kFFCircularBufferSize] = kFFToAddToBuffer;
      kFFCircularBufferCounter++;
    }

    if (shooterControlState == ShooterControlState.HOLD) {
      // set shooter pid values to 0
      shooterPID.setP(0);
      shooterPID.setI(0);
      shooterPID.setD(0);

      // replace with calculated kF
      Arrays.sort(kFFCircularBuffer);
      double medianKFF = kFFCircularBuffer[ShooterConstants.kFFCircularBufferSize/2];
      shooterPID.setFF(medianKFF); // get median value

      SetShooterSpeed(desiredRPM);
    }

  }
  public void PIDTuner() {
    double pTemp = 0;
    double iTemp = 0;
    double dTemp = 0;
    double ffTemp = 0;
    boolean valueOrPID;

    pTemp = SmartDashboard.getNumber("Current pVal = ", 0);
    iTemp = SmartDashboard.getNumber("Current iVal = ", 0);
    dTemp = SmartDashboard.getNumber("Current dVal = ", 0);
    ffTemp = SmartDashboard.getNumber("Current ffVal = ", 0);
    motor1ShooterSpeed = SmartDashboard.getNumber("Shooter Speed = ", 0); 
    desiredRPM = SmartDashboard.getNumber("Desired RPM = ", 0);
    valueOrPID = SmartDashboard.getBoolean("Value or SetPID:", true);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((pTemp != kP)) { shooterPID.setP(pTemp); kP = pTemp; }
    if((kI != iTemp)) { shooterPID.setI(iTemp); kI = iTemp; }
    if((dTemp != kD)) { shooterPID.setD(dTemp); kD = dTemp; }
    if((ffTemp != kFF)) { shooterPID.setFF(ffTemp); kFF = ffTemp; }
   

    if (motor1ShooterSpeed >= 1.0) {
      motor1ShooterSpeed = 1.0;
    } else if (motor1ShooterSpeed <= -1.0) {
      motor1ShooterSpeed = -1.0;
    }

    if (valueOrPID) {
      leftShooterMotor.set(motor1ShooterSpeed);
      //SMotorDale.set(-motor1ShooterSpeed);
      SmartDashboard.putNumber("Output Chip", leftShooterMotor.getAppliedOutput());
      SmartDashboard.putNumber("Output Dale", rightShooterMotor.getAppliedOutput());
    } else {
      // Use PID value
      shooterPID.setReference(desiredRPM, ControlType.kVelocity);
      //SmartDashboard.putNumber("Shooter Vel", shooterEncoder.getVelocity());
      SmartDashboard.putNumber("Output Chip", leftShooterMotor.getAppliedOutput());
      SmartDashboard.putNumber("Output Dale", rightShooterMotor.getAppliedOutput());
    }
  }

  /**
   * Determine RPM of shooter needed to score power cells in power port 
   * xdist - distance from power port in m from vision processing
   * fudgeFactor - multiplication needed to turn velocity into RPM
   */
  public double getNeededRPM() {

    double vel, RPM;

    // Convert xdist to feet
    double xDist_ft = x * GenConstants.M_TO_FEET;
   
    // Ya know math 
    vel = (xDist_ft/Constants.GenConstants.COS_ANGLE)
      *Math.pow(-Constants.GenConstants.G_FT_PER_SEC2/
      (Constants.GenConstants.INNER_PORT_HEIGHT_FT-Constants.GenConstants.TAN_ANGLE*xDist_ft
      -Constants.GenConstants.SHOOTER_HEIGHT_FT),0.5);

    RPM = fudgeFactor*vel;

    SmartDashboard.putNumber("Calculated velocity", vel);
    SmartDashboard.putNumber("Calculated RPM", RPM);

    return RPM;
  }
  
  public void SetShooterSpeed(double speed) {
    shooterPID.setReference(speed, ControlType.kVelocity);
    //double voltage = 1.02e-3*speed + 0.459;
    //leftShooterMotor.setVoltage(voltage); //manually set motor speed (voltage), negative shoots
    
    // double shooterSpeed = ShooterEncoder.getVelocity();
    // boolean isDone = shooterSpeed > speed;
    // return isDone;
  }

  public boolean AtSpeed(double referenceSpeed) {
    double shooterSpeed = shooterEncoder.getVelocity();
    boolean isDone = Math.abs(shooterSpeed) > Math.abs(referenceSpeed);
    return isDone;
  }

  public double kFcalculator(double dutyCycle, double rpm) {
    if (Math.abs(rpm) < 0.1) {
      return 0;
    }
    //double kf = (1023.0/12.0) * voltage / ((4096.0/600.0) * rpm);
    double kf = dutyCycle / ((4096.0/600.0) * rpm);
    return kf;
  }

  public void stopFlywheel(){
    leftShooterMotor.setVoltage(0);
  }

  public boolean getReadyToFire() {
    return readyToFire;
  }

  public void setReadyToFire(boolean newValue) {
    readyToFire = newValue;
  }

  public ShooterControlState getShooterState() {
    return this.shooterControlState;
  }

  public void setShooterState(ShooterControlState shooterControlState) {
    this.shooterControlState = shooterControlState;
  }

  private void resetHold() {
    Arrays.fill(kFFCircularBuffer, 0.0);
    kFFSampleCount = 0;
    onTarget = false;
  }
}
