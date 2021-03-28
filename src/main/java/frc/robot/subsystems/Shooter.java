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
import edu.wpi.first.wpilibj.Relay;
/*
** PURPOSE: Shooter subsystem
** STATUS: Motor functions are good. PID is not.
** I believe network table connection was working (we were getting values from the pi),
** but not sure if things have changed since chameleon vision changed
*/
public class Shooter extends SubsystemBase {
  private CANSparkMax mLeftShooterMotor;
  private CANSparkMax mRightShooterMotor;
  private double mDesiredRPM = 0;
  private double mAbsoluteRPMError = 0;
  private double mLeftShooterMotorSpeed = 0;
  private double mLeftShooterRPM = 0.0;
  private double mLeftShooterVoltage = 0.0;
  private double mLeftShooterDutyCycle = 0.0;
  private Relay  mPVRingLight;

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
  private double mPVVel = 0;
  private double mPVRPM = 0;
  private double mMedianKFF = 0;
  private boolean mSetKFF = true;

  // Network table for chameleon vision
  NetworkTableInstance table = NetworkTableInstance.getDefault();
  NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("PsThreeCam");
  private NetworkTableEntry targetPose;
  private double targetArr[] = {0, 0, 0};
  private double x, y, angle = 0;
  private double fudgeFactor = 0;
  private boolean readyToFire = false;
  private ShooterControlState shooterControlState = ShooterControlState.IDLE;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {

    // Set up motors
    mLeftShooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_LEFT, MotorType.kBrushed);
    mRightShooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_RIGHT, MotorType.kBrushed);
    mPVRingLight = new Relay(ShooterConstants.PV_RING_LIGHT);
    mLeftShooterMotor.restoreFactoryDefaults();
    mRightShooterMotor.restoreFactoryDefaults();
    mLeftShooterMotor.setSmartCurrentLimit(30);
    mRightShooterMotor.setSmartCurrentLimit(30);
    mLeftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mRightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mLeftShooterMotor.burnFlash();
    mRightShooterMotor.burnFlash();
    // Set Right to follow Left, but inverted
    mRightShooterMotor.follow(mLeftShooterMotor, true);

    // Instantiate encoder
    shooterEncoder = mLeftShooterMotor.getEncoder(EncoderType.kQuadrature, GenConstants.REV_ENCODER_CPR);

    // Start PID
    shooterPID = mLeftShooterMotor.getPIDController();
    shooterPID.setFeedbackDevice(shooterEncoder);
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(-1,1);
    shooterPID.setIZone(1000);

  }

  @Override
  public void periodic() {

    // Get distance data from camera
    targetPose = cameraTable.getEntry("targetPose");
    targetArr = targetPose.getDoubleArray(targetArr);

    // Save the most recent data
    x = targetArr[0];
    y = targetArr[1];
    angle = targetArr[2];

    // PIDTuner(); // Comment this out once we figure out our PID values.
    // getNeededRPM();
    closedLoopStateMachineManager();
    updateSmartdashboard();
  }

  public void closedLoopStateMachineManager(){
    mLeftShooterRPM = shooterEncoder.getVelocity();
    mLeftShooterVoltage = mLeftShooterMotor.getBusVoltage();
    mLeftShooterDutyCycle = mLeftShooterMotor.getAppliedOutput();

    // State Machine
    if (shooterControlState == ShooterControlState.IDLE) {
      kP = 0.0005;
      kI = 0.0000025;
      kD = 0;
      kFF = 0.000013;

      shooterPID.setP(kP);
      shooterPID.setI(kI);
      shooterPID.setD(kD);
      shooterPID.setFF(kFF);

      // System.out.println("Setting Default kF");
      mLeftShooterMotor.set(0);
      mSetKFF = true;
      mPVRingLight.set(Relay.Value.kOff);


    } else if (shooterControlState == ShooterControlState.SPINUP) {
      // figure out desired speed
      // desiredRPM = getNeededRPM(); // for when we have camera control
      // pid spin up
      setShooterSpeed(mDesiredRPM);
      mPVRingLight.set(Relay.Value.kOn);


      shooterControlState = ShooterControlState.HOLDWHENREADY;

    } else if (shooterControlState == ShooterControlState.HOLDWHENREADY) {
      mAbsoluteRPMError = Math.abs(mDesiredRPM - mLeftShooterRPM);

      if (mAbsoluteRPMError < 100 && !onTarget) {
        onTarget = true;
      } else if (mAbsoluteRPMError > 100) {
        resetHold(); // onTarget is now false
      }

      if (onTarget) {
        // record value in kFFCircularBuffer
        kFFCircularBuffer[kFFCircularBufferCounter % ShooterConstants.kFFCircularBufferSize] = kFcalculator(mLeftShooterVoltage, mLeftShooterRPM);
        kFFCircularBufferCounter++;
        kFFSampleCount++;
      }

      if (kFFSampleCount >= ShooterConstants.kFFCircularBufferSize) {
        shooterControlState = ShooterControlState.HOLD;
      } else {
        setShooterSpeed(mDesiredRPM);
      }

      double kFFToAddToBuffer = kFcalculator(mLeftShooterMotor.getBusVoltage(), shooterEncoder.getVelocity());
      kFFCircularBuffer[kFFCircularBufferCounter % ShooterConstants.kFFCircularBufferSize] = kFFToAddToBuffer;
      kFFCircularBufferCounter++;
    }

    if (shooterControlState == ShooterControlState.HOLD) {
      // set shooter pid values to 0

      if (mSetKFF){
        shooterPID.setP(0);
        shooterPID.setI(0);
        shooterPID.setD(0);
  
        // replace with calculated kF
        Arrays.sort(kFFCircularBuffer);
        mMedianKFF = kFFCircularBuffer[ShooterConstants.kFFCircularBufferSize/2];
        shooterPID.setFF(mMedianKFF); // get median value
        System.out.print("Setting Calculated kF");
        System.out.println(mMedianKFF);
        mSetKFF = false;
      }
      setShooterSpeed(mDesiredRPM);
    }
  }

  public void PIDTuner() {
    double pTemp = 0;
    double iTemp = 0;
    double dTemp = 0;
    double ffTemp = 0;
    boolean valueOrPID;

    pTemp = SmartDashboard.getNumber("[Shooter] Current pVal = ", 0);
    iTemp = SmartDashboard.getNumber("[Shooter] Current iVal = ", 0);
    dTemp = SmartDashboard.getNumber("[Shooter] Current dVal = ", 0);
    ffTemp = SmartDashboard.getNumber("[Shooter] Current ffVal = ", 0);
    mLeftShooterMotorSpeed = SmartDashboard.getNumber("[Shooter] Speed = ", 0);
    mDesiredRPM = SmartDashboard.getNumber("[Shooter] Desired RPM = ", 0);
    valueOrPID = SmartDashboard.getBoolean("[Shooter] Value or SetPID:", true);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if (pTemp != kP) { shooterPID.setP(pTemp); kP = pTemp; }
    if (kI != iTemp) { shooterPID.setI(iTemp); kI = iTemp; }
    if (dTemp != kD) { shooterPID.setD(dTemp); kD = dTemp; }
    if (ffTemp != kFF) { shooterPID.setFF(ffTemp); kFF = ffTemp; }

    if (mLeftShooterMotorSpeed >= 1.0) {
      mLeftShooterMotorSpeed = 1.0;
    } else if (mLeftShooterMotorSpeed <= -1.0) {
      mLeftShooterMotorSpeed = -1.0;
    }

    if (valueOrPID) {
      mLeftShooterMotor.set(mLeftShooterMotorSpeed); // right shooter motor follows left
      SmartDashboard.putNumber("Output Left Shooter", mLeftShooterMotor.getAppliedOutput());
      SmartDashboard.putNumber("Output Right Shooter", mRightShooterMotor.getAppliedOutput());
    } else {
      // Use PID value
      shooterPID.setReference(mDesiredRPM, ControlType.kVelocity);
      // SmartDashboard.putNumber("Shooter Vel", shooterEncoder.getVelocity());
      SmartDashboard.putNumber("Output Left Shooter", mLeftShooterMotor.getAppliedOutput());
      SmartDashboard.putNumber("Output Right Shooter", mRightShooterMotor.getAppliedOutput());
    }
  }

  /**
   * Determine RPM of shooter needed to score power cells in power port
   * xdist - distance from power port in m from vision processing
   * fudgeFactor - multiplication needed to turn velocity into RPM
   */
  public double getNeededRPM() {

    // Convert xdist to feet
    double xDist_ft = x * GenConstants.M_TO_FEET;

    // TODO confirm math
    mPVVel = (xDist_ft/Constants.GenConstants.COS_ANGLE)
      * Math.pow(-Constants.GenConstants.G_FT_PER_SEC2
      / (Constants.GenConstants.INNER_PORT_HEIGHT_FT - Constants.GenConstants.TAN_ANGLE * xDist_ft
      - Constants.GenConstants.SHOOTER_HEIGHT_FT), 0.5);

    mPVRPM = fudgeFactor * 2 * mPVVel / (Math.PI * Constants.ShooterConstants.SHOOTER_FLYWHEEL_DIAMETER); // COM PC velocity is half surface velocity 

    return mPVRPM;
  }

  public double getDesiredRPM() {
    return mDesiredRPM;
  }

  public void setDesiredRPM(double desiredRPM) {
    this.mDesiredRPM = desiredRPM;
  }

  public void setShooterSpeed(double speed) {
    shooterPID.setReference(speed, ControlType.kVelocity);
    // double voltage = 1.02e-3*speed + 0.459;
    // leftShooterMotor.setVoltage(voltage); //manually set motor speed (voltage), negative shoots
  }

  public boolean atSpeed(double referenceSpeed) {
    double shooterSpeed = shooterEncoder.getVelocity();
    boolean isDone = Math.abs(shooterSpeed) > Math.abs(referenceSpeed);
    return isDone;
  }

  public double kFcalculator(double dutyCycle, double rpm) {
    if (Math.abs(rpm) < 0.1) {
      return 0;
    }
    // double kf = (1023.0/12.0) * voltage / ((4096.0/600.0) * rpm);
    double kf = dutyCycle / ((4096.0/600.0) * rpm);
    return kf;
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

  private void updateSmartdashboard(){
    SmartDashboard.putNumber("[Shooter] Calculated velocity", mPVVel);
    SmartDashboard.putNumber("[Shooter] Calculated RPM", mPVRPM);
    SmartDashboard.putNumber("[Shooter] kF", kFcalculator(mLeftShooterDutyCycle, mLeftShooterRPM));
    SmartDashboard.putNumber("[Shooter] Encoder Position", shooterEncoder.getPosition());
    SmartDashboard.putNumber("[Shooter] Encoder Velocity", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("[Shooter] Current pVal = ", kP);
    SmartDashboard.putNumber("[Shooter] Current iVal = ", kI);
    SmartDashboard.putNumber("[Shooter] Current dVal = ", kD);
    SmartDashboard.putNumber("[Shooter] Current ffVal = ", kFF);
    SmartDashboard.putNumber("[Shooter] Speed = ", mLeftShooterMotorSpeed);
    SmartDashboard.putNumber("[Shooter] Desired RPM = ", mDesiredRPM);
    SmartDashboard.putBoolean("[Shooter] Value or SetPID:", true); // Set to true for using "Shooter Motor Speed" to control shooter speed
    SmartDashboard.putNumber("[Shooter] Calculated velocity", 0);
    SmartDashboard.putNumber("[Shooter] Calculated RPM", 0);
    SmartDashboard.putNumber("[Shooter] Camera X Dist", x);
    SmartDashboard.putNumber("[Shooter] Camera Y Dist", y);
    SmartDashboard.putNumber("[Shooter] Camera Angle Dist", angle);

    SmartDashboard.putNumber("[Shooter] Fudge Factor", fudgeFactor);
    SmartDashboard.getNumber("[Shooter] Fudge Factor", fudgeFactor);

    SmartDashboard.putNumber("[Shooter] Camera X Dist", x);
    SmartDashboard.putNumber("[Shooter] Camera Y Dist", y);
    SmartDashboard.putNumber("[Shooter] Camera Angle Dist", angle);
    SmartDashboard.putString("[Shooter] State", shooterControlState.toString());
    String median_kff = String.format("%e", kFF);
    SmartDashboard.putString("[Shooter] Calculated kFF", median_kff);
  }
}
