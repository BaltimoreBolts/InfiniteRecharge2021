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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANError;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants;
import frc.robot.Globals.PCArray;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.AlternateEncoderType;
import java.lang.Math;

/**
 * PURPOSE: Indexer subsystem STATUS: There's a lot of functions in here,
 * probably 50% tested Bang bang controller for indexer rotation works if speed
 * is set low enough. PID might work now? I just added the magic line
 */
public class Indexer extends SubsystemBase {
  private CANSparkMax mIndexerMotor;
  private CANPIDController mIndexerPID;
  private double mCommandPos = 0;

  // private DigitalInput OpticalSensor;
  private TimeOfFlight mIndexerTOF;
  private CANEncoder mAlternateEncoder;
  private CANEncoder mMainEncoder;

  ShuffleboardTab indexerTab;
  NetworkTableEntry mDesiredRotationNT, mCurrentRotationNT, mDesiredSpeedNT;
  private double mIndexerSpeed = 0;
  double mOverShoot;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {

    // initialize motor
    mIndexerMotor = new CANSparkMax(IndexerConstants.INDEXER_MOTOR, MotorType.kBrushless);
    mIndexerMotor.restoreFactoryDefaults();
    mIndexerMotor.setSmartCurrentLimit(30);
    mIndexerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mIndexerMotor.burnFlash();

    // initialize sensors
    mIndexerTOF = new TimeOfFlight(IndexerConstants.INDEXER_TOF);

    // initialize encoders
    mMainEncoder = mIndexerMotor.getEncoder();
    mAlternateEncoder = mIndexerMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature,
        Constants.GenConstants.REV_ENCODER_CPR);
    mAlternateEncoder.setInverted(true); // We need this!

    /*
     * By default, the PID controller will use the Hall sensor from a NEO or NEO 550
     * for its feedback device. Instead, we can set the feedback device to the
     * alternate encoder object
     */
    mIndexerPID = mIndexerMotor.getPIDController();

    CANError pid_error = mIndexerPID.setFeedbackDevice(mMainEncoder);
    if (pid_error != CANError.kOk) {
      SmartDashboard.putString("[Indexer] PID Error", pid_error.toString()); //TODO remove when not testing
    }

    mIndexerPID.setP(IndexerConstants.kP);
    mIndexerPID.setI(IndexerConstants.kI);
    mIndexerPID.setD(IndexerConstants.kD);
    // indexerPID.setFF(IndexerConstants.kFF);
    mIndexerPID.setOutputRange(-0.2, 0.2); // output range for motor power
    mIndexerPID.setIZone(1); // only use integral constant within 1 degree error

    // int smartMotionSlot = 0;
    // indexerPID.setSmartMotionMaxVelocity(7000, smartMotionSlot);
    // indexerPID.setSmartMotionMinOutputVelocity(-7000, smartMotionSlot);
    // indexerPID.setSmartMotionMaxAccel(1400, smartMotionSlot);
    // indexerPID.setSmartMotionAllowedClosedLoopError(0.1, smartMotionSlot);

    // ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");
    // mDesiredRotationNT = indexerTab.add("Desired Rotation = ", 0).getEntry();
    // mCurrentRotationNT = indexerTab.add("Current Rotation = ", 0).getEntry();
    // mDesiredSpeedNT = indexerTab.add("Desired Speed = ", 0).getEntry();

    this.resetEncoder(); // TODO if encoder gives absolute values do we need this?
    SmartDashboard.setDefaultBooleanArray("[Indexer] PC Array", PCArray.getPCArray());

  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    
  //   if (p != IndexerConstants.kP) {
  //     indexerPID.setP(p);
  //   }
  //   if (i != IndexerConstants.kI) {
  //     indexerPID.setI(i);
  //   }
  //   if (d != IndexerConstants.kD) {
  //     indexerPID.setD(d);
  //   }
    updateSmartdashboard();
  }

  /*
  public void powerCellStateMachineManager() {
    switch (PowerCellConstants.powerCellState) {
      case IDLE:
        break;

      case HARVESTING:
        // (check if already full?)
        // if we have balls already they go down
        // then take in ball
        // harvester rotate
        // tof check
        // indexer move
        break;

      case SHOOTING:
        // (make sure we have a ball?)
        // move all balls to the top
        // index up to shoot
        break;

      case PURGING:
        break;

      default:
        System.out.println("[ERROR] This state should never occur.");
        PowerCellConstants.powerCellState = PowerCellConstants.powerCellStates.IDLE;
    }
  }
  */

  public int degreeToCounts(double degrees, int CPR) {
    return (int) Math.ceil(CPR * degrees / 360.0);
  }

  // Move the indexer motor at a certain speed
  public void setIndexerSpeed(double speed) {
    mIndexerMotor.set(speed);
  }

  public boolean moveToPosition(double desiredPosition, double resetDistance) {
    // Pass in the position you want to be in, returns true if you're there
    // Supposed to use resetDistance to put on exact 1/3 turn locations everytime :) WIP
    // resetDistance uses abs encoder to correct for absolute position
    this.mCommandPos = desiredPosition + 70.0 * resetDistance; // 70:1 gearbox between rel.enc and output
    CANError pidError = this.mIndexerPID.setReference(this.mCommandPos, ControlType.kPosition);
    SmartDashboard.putString("[Indexer] Move Position PID Error", pidError.toString());
    return Math.abs(this.getEncoderValue() - this.mCommandPos) < 0.1;
  }

  public double getEncoderValue() {
    // return alternateEncoder.getPosition();
    // this returns built-in motor controller position
    return mIndexerMotor.getEncoder().getPosition();
  }

  public double getAbsEncoderValue() {
    return mIndexerMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192).getPosition();
  }

  // Return value of first position optical sensor
  // If indexer TOF sees PC, this returns true. These values likely need to be adjusted
  public boolean checkBottomTOF() {
    double distance = mIndexerTOF.getRange(); // in mm
    // Because the ball is curved we want to stop when the center of the ball is in
    // front of the sensor hence the range
    return distance >= 15 && distance <= 25;
  }

  public void resetEncoder() {
    mMainEncoder.setPosition(0);
    // alternateEncoder.setPosition(0);
  }

  // Functions for compenstating based on indexer overshoot. We never tested these
  public void calculateOvershoot(double encoderVal, double desiredPosition) {
    mOverShoot = encoderVal - desiredPosition;
  }

  public double getOvershoot() {
    return mOverShoot;
  }

  // TESTING FUNCTIONS
  // For testing, this will be disabled later
  public double getDesiredSpeed() {
    return mIndexerSpeed;
  }
  
  private void updateSmartdashboard(){
    SmartDashboard.putNumber("[Indexer] Speed", mIndexerSpeed);
    SmartDashboard.putNumber("[Indexer] P Gain", IndexerConstants.kP);
    SmartDashboard.putNumber("[Indexer] I Gain", IndexerConstants.kI);
    SmartDashboard.putNumber("[Indexer] D Gain", IndexerConstants.kD);
    // SmartDashboard.putNumber("Set Rotations", 0); // for PID from smartdashboard
    SmartDashboard.putBoolean("[Indexer] TOF", this.checkBottomTOF());
    SmartDashboard.putNumber("[Indexer] TOF Val", mIndexerTOF.getRange());
    SmartDashboard.putNumber("[Indexer] Encoder", this.getEncoderValue());
    SmartDashboard.putNumber("[Indexer] Motor Encoder", mIndexerMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("[Indexer] Motor Alternate Encoder",
      mIndexerMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192).getPosition());
    SmartDashboard.putNumber("[Indexer] Overshoot", mOverShoot);
    SmartDashboard.putNumber("[Indexer] Command Position", mCommandPos);
    SmartDashboard.putBooleanArray("[Indexer] PC Array", PCArray.getPCArray());
    boolean[] tempPCArray = PCArray.getPCArray();
    String tempPCString = String.format("[ 0: %b | 1: %b | 2: %b ]", tempPCArray[0], tempPCArray[1], tempPCArray[2]);
    SmartDashboard.putString("[Indexer] PC Array", tempPCString);
    //SmartDashboard.putString("[Indexer] PC Positions", PCArray.toString())
    // PCArray.putPCArray(SmartDashboard.getBooleanArray("[Indexer] PC Array", new boolean[3]));

    mIndexerSpeed = SmartDashboard.getNumber("[Indexer] Speed", 0);
    // double p = SmartDashboard.getNumber("[Indexer] P Gain", 0.07);
    // double i = SmartDashboard.getNumber("[Indexer] I Gain", 0.003);
    // double d = SmartDashboard.getNumber("[Indexer] D Gain", 0);

  } 
}

