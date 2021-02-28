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
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.AlternateEncoderType;
import java.lang.Math;

/**
 * PURPOSE: Indexer subsystem STATUS: There's a lot of functions in here,
 * probably 50% tested Bang bang controller for indexer rotation works if speed
 * is set low enough. PID might work now? I just added the magic line
 */
public class Indexer extends SubsystemBase {
  private CANSparkMax indexerMotor;
  private CANPIDController indexerPID;
  private double kP = 0.07; // 2e-5 initial test value
  private double kI = 0.03; // 0 initial test value
  private double kD = 0.0; // had success 0.03 but need to check
  private double kFF = 0;
  private double commandPos = 0;

  // private DigitalInput OpticalSensor;
  private TimeOfFlight IndexerTOF;
  private CANEncoder alternateEncoder;
  private CANEncoder mainEncoder;

  ShuffleboardTab indexerTab;
  NetworkTableEntry desiredRotationNT, currentRotationNT, desiredSpeedNT;
  private double indexerSpeed = 0;
  double overShoot;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {

    // initialize motor
    indexerMotor = new CANSparkMax(IndexerConstants.INDEXER_MOTOR, MotorType.kBrushless);
    indexerMotor.restoreFactoryDefaults();
    indexerMotor.setSmartCurrentLimit(30);
    indexerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    indexerMotor.burnFlash();

    // initialize sensors
    IndexerTOF = new TimeOfFlight(IndexerConstants.INDEXER_TOF);

    // initialize encoders
    mainEncoder = indexerMotor.getEncoder();
    alternateEncoder = indexerMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature,
        Constants.GenConstants.REV_ENCODER_CPR);
    alternateEncoder.setInverted(true); // We need this!

    /*
     * By default, the PID controller will use the Hall sensor from a NEO or NEO 550
     * for its feedback device. Instead, we can set the feedback device to the
     * alternate encoder object
     */
    indexerPID = indexerMotor.getPIDController();

    CANError pid_error = indexerPID.setFeedbackDevice(mainEncoder);
    if (pid_error != CANError.kOk) {
      SmartDashboard.putString("[Indexer] PID Error", pid_error.toString());
    }

    indexerPID.setP(kP);
    indexerPID.setI(kI);
    indexerPID.setD(kD);
    // indexerPID.setFF(kFF);
    indexerPID.setOutputRange(-0.5, 0.5);
    indexerPID.setIZone(1);

    // int smartMotionSlot = 0;
    // indexerPID.setSmartMotionMaxVelocity(7000, smartMotionSlot);
    // indexerPID.setSmartMotionMinOutputVelocity(-7000, smartMotionSlot);
    // indexerPID.setSmartMotionMaxAccel(1400, smartMotionSlot);
    // indexerPID.setSmartMotionAllowedClosedLoopError(0.1, smartMotionSlot);

    ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");
    desiredRotationNT = indexerTab.add("Desired Rotation = ", 0).getEntry();
    currentRotationNT = indexerTab.add("Current Rotation = ", 0).getEntry();
    desiredSpeedNT = indexerTab.add("Desired Speed = ", 0).getEntry();

    SmartDashboard.putNumber("[Indexer] Speed", indexerSpeed);
    SmartDashboard.putNumber("[Indexer] P Gain", kP);
    SmartDashboard.putNumber("[Indexer] I Gain", kI);
    SmartDashboard.putNumber("[Indexer] D Gain", kD);
    // SmartDashboard.putNumber("Set Rotations", 0); // for PID from smartdashboard

    this.resetEncoder(); // TODO if encoder gives absolute values do we need this?
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    
    // print statements
    SmartDashboard.putBoolean("[Indexer] TOF", this.checkBottomTOF());
    SmartDashboard.putNumber("[Indexer] TOF Val", IndexerTOF.getRange());
    SmartDashboard.putNumber("[Indexer] Encoder", this.getEncoderValue());
    SmartDashboard.putNumber("[Indexer] Motor Encoder", indexerMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("[Indexer] Motor Alternate Encoder",
      indexerMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192).getPosition());
    SmartDashboard.putNumber("[Indexer] Overshoot", overShoot);
    SmartDashboard.putNumber("[Indexer] Command Position", commandPos);

    indexerSpeed = SmartDashboard.getNumber("[Indexer] Speed", 0);
    double p = SmartDashboard.getNumber("[Indexer] P Gain", 0.07);
    double i = SmartDashboard.getNumber("[Indexer] I Gain", 0.003);
    double d = SmartDashboard.getNumber("[Indexer] D Gain", 0);

    if (p != kP) {
      indexerPID.setP(p);
      kP = p;
    }
    if (i != kI) {
      indexerPID.setI(i);
      kI = i;
    }
    if (d != kD) {
      indexerPID.setD(d);
      kD = d;
    }
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
    indexerMotor.set(speed);
  }

  public boolean moveToPosition(double desiredPosition, double resetDistance) {
    // Pass in the position you want to be in, returns true if you're there
    // Supposed to use resetDistance to put on exact 1/3 turn locations everytime :) WIP
    // resetDistance uses abs encoder to correct for absolute position
    this.commandPos = desiredPosition + 70.0 * resetDistance; // 70:1 gearbox between rel.enc and output
    CANError pidError = this.indexerPID.setReference(this.commandPos, ControlType.kPosition);
    SmartDashboard.putString("[Indexer] Move Position PID Error", pidError.toString());
    return Math.abs(this.getEncoderValue() - this.commandPos) < 0.1;
  }

  public double getEncoderValue() {
    // return alternateEncoder.getPosition();
    // this returns built-in motor controller position
    return indexerMotor.getEncoder().getPosition();
  }

  public double getAbsEncoderValue() {
    return indexerMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192).getPosition();
  }

  // Return value of first position optical sensor
  // If indexer TOF sees PC, this returns true. These values likely need to be adjusted
  public boolean checkBottomTOF() {
    double distance = IndexerTOF.getRange(); // in mm
    // Because the ball is curved we want to stop when the center of the ball is in
    // front of the sensor hence the range
    return distance >= 15 && distance <= 25;
  }

  public void resetEncoder() {
    mainEncoder.setPosition(0);
    // alternateEncoder.setPosition(0);
  }

  // Functions for compenstating based on indexer overshoot. We never tested these
  public void calculateOvershoot(double encoderVal, double desiredPosition) {
    overShoot = encoderVal - desiredPosition;
  }

  public double getOvershoot() {
    return overShoot;
  }

  // TESTING FUNCTIONS
  // For testing, this will be disabled later
  public double getDesiredSpeed() {
    return desiredSpeedNT.getDouble(0);
  }
}
