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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.AlternateEncoderType;
import java.lang.Math;

/* 
** PURPOSE: Indexer subsystem 
** STATUS: There's a lot of functions in here, probably 50% tested
** Bang bang controller for indexer rotation works if speed is set low enough. 
** PID might work now? I just added the magic line
*/

public class Indexer extends SubsystemBase {
  private CANSparkMax IndexerDonaldMotor;
  private CANPIDController indexerPID;
  private double kP = 0.07; // 2e-5 initial test value 
  private double kI = 0.03; // 0 initial test value 
  private double kD = 0.0; // had success 0.03 but need to check
  private double kFF = 0; 
  private double commandPos = 0;

  //private DigitalInput OpticalSensor;
  private TimeOfFlight IndexerTOF;
  private CANEncoder alternateEncoder;
  private CANEncoder mainEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private boolean PCArray[] = {false,false,false,false};


  ShuffleboardTab indexerTab;
  NetworkTableEntry desiredRotationNT, currentRotationNT, desiredSpeedNT;
  NetworkTableEntry PCDash0, PCDash1, PCDash2, PCDash3;
  boolean shiftIndexer = false; 
  private double indexerSpeed = 0;
  double overShoot;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    IndexerDonaldMotor = new CANSparkMax (IndexerConstants.INDEXER_MOTOR_DONALD, MotorType.kBrushless);
    IndexerDonaldMotor.restoreFactoryDefaults();
    IndexerDonaldMotor.setSmartCurrentLimit(30);
    IndexerDonaldMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); 
    // OpticalSensor = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH1);
    IndexerTOF = new TimeOfFlight(IndexerConstants.INDEXER_TOF);
    alternateEncoder = IndexerDonaldMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature,
            Constants.GenConstants.REV_ENCODER_CPR);
    alternateEncoder.setInverted(true); //We do need this
    mainEncoder = IndexerDonaldMotor.getEncoder();
                        
    IndexerDonaldMotor.burnFlash();
    //PCArray= {false,false,false,false};
    PCArray[0] = false;
    PCArray[1] = false;
    PCArray[2] = false;
    PCArray[3] = false;

    double overShoot = 0;

    indexerPID = IndexerDonaldMotor.getPIDController();
  /*
  * By default, the PID controller will use the Hall sensor from a NEO or NEO 550 for
  * its feedback device. Instead, we can set the feedback device to the alternate
  * encoder object
  */
  
  CANError pid_error = indexerPID.setFeedbackDevice(mainEncoder);
    if (pid_error != CANError.kOk) {
      SmartDashboard.putString("PID", pid_error.toString());
    }

    indexerPID.setP(kP);
    indexerPID.setI(kI);
    indexerPID.setD(kD); 
    //indexerPID.setFF(kFF);
    indexerPID.setOutputRange(-0.1, 0.1);  
    indexerPID.setIZone(1);
    
    /*
    int smartMotionSlot = 0;
    indexerPID.setSmartMotionMaxVelocity(7000, smartMotionSlot);
    indexerPID.setSmartMotionMinOutputVelocity(-7000, smartMotionSlot);
    indexerPID.setSmartMotionMaxAccel(1400, smartMotionSlot);
    indexerPID.setSmartMotionAllowedClosedLoopError(0.1, smartMotionSlot);
    */

    ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");
    desiredRotationNT = indexerTab.add("Desired Rotation = ", 0).getEntry();
    currentRotationNT = indexerTab.add("Current Rotation = ", 0).getEntry();
    desiredSpeedNT = indexerTab.add("Desired Speed = ", 0).getEntry();
    //PCIndicator = indexerTab.add("Power Cell Array",0).getEntry();
    PCDash0 = indexerTab.add("PC0",PCArray[0]).getEntry();
    PCDash1 = indexerTab.add("PC1",PCArray[1]).getEntry();
    PCDash2 = indexerTab.add("PC2",PCArray[2]).getEntry();
    PCDash3 = indexerTab.add("PC3",PCArray[3]).getEntry();

    //indexerSpeed = 0; // Debug stuff 
    SmartDashboard.putNumber("Indexer Speed", indexerSpeed);
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    //SmartDashboard.putNumber("Set Rotations", 0); //for PID from smartdashboard

    this.ResetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    UpdateDashboard();
  
    SmartDashboard.putBoolean("Indexer TOF", this.getP0());
    SmartDashboard.putNumber("Indexer TOF Val", IndexerTOF.getRange());
    SmartDashboard.putNumber("Indexer Encoder", this.getEncoderValue());
    SmartDashboard.putNumber("Motor Encoder", IndexerDonaldMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Motor Alternate Encoder" , IndexerDonaldMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192).getPosition()); 
    SmartDashboard.putBooleanArray("Indexer Array", PCArray);
    indexerSpeed = SmartDashboard.getNumber("Indexer Speed", 0);
    SmartDashboard.putNumber("Indexer Overshoot", overShoot);
    SmartDashboard.putNumber("Command Position", commandPos);

    double p = SmartDashboard.getNumber("P Gain", 0.07);
    double i = SmartDashboard.getNumber("I Gain", 0.003);
    double d = SmartDashboard.getNumber("D Gain", 0);

    if((p != kP)) { indexerPID.setP(p); kP = p; }
    if((i != kI)) { indexerPID.setI(i); kI = i; }
    if((d != kD)) { indexerPID.setD(d); kD = d; }
  }

  public int degreeToCounts(double degrees, int CPR ){
    int Counts = 0;
    Counts = (int)Math.ceil(CPR * degrees/360.0);
    return Counts;

  }
  
  /*Publish values we want to look at to dashboard */
  public void UpdateDashboard() {
    //currentRotationNT.setDouble(alternateEncoder.getPosition());
    desiredSpeedNT.getDouble(0);
    desiredRotationNT.getDouble(0);
  } 

  // Shift array locations up
  public void ShiftPCArray(boolean PC0) {
    PCArray[3] = PCArray[2];
    PCArray[2] = PCArray[1];
    PCArray[1] = PCArray[0];
    PCArray[0] = PC0;
  }

  // Initialize PC array.
  public void SetPCArray(boolean[] inputArray){
    PCArray[0] = inputArray[0];
    PCArray[1] = inputArray[1];
    PCArray[2] = inputArray[2];
    PCArray[3] = inputArray[3];
   // PCArray[0] = OpticalSensor.get();
    PCArray [0] = this.getP0();
  }

  //Move the indexer motor at a certain speed
  public void Movement (double speed){
    IndexerDonaldMotor.set(speed);
  } 

  public boolean MoveToPosition(double desiredPosition, double resetDistance) {
    // Pass in the position you want to be in, return true if you're there
    // resetDistance uses abs encoder to correct for absolute position 

    this.commandPos = desiredPosition + 70.0*resetDistance; //70:1 gearbox between rel.enc and output
    CANError pidError = this.indexerPID.setReference(this.commandPos, ControlType.kVelocity); 
    SmartDashboard.putString("PID Error", pidError.toString());
    boolean isDone = Math.abs(this.getEncoderValue() - this.commandPos) < 0.1;// | IndexerDonaldMotor.getEncoder().getVelocity() < 0.001;
    return isDone;
  }

  public double getEncoderValue(){
    //return alternateEncoder.getPosition();
    // this returns built-in motor controller position
    return IndexerDonaldMotor.getEncoder().getPosition();
  }

  public double getAbsEncoderValue(){
    return IndexerDonaldMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192).getPosition();
  }

  //Return value of first position optical sensor
  // If indexer TOF sees PC, this returns true. These values likely need to be adjusted
  public boolean getP0(){
    double distance = IndexerTOF.getRange(); // in mm
    //return OpticalSensor.get();
    // Because the ball is curved we want to stop when the center of the ball is in front of the sensor hence the range 
    
    return distance >= 15 && distance <= 25;
  }

  public boolean isIndexerFull() {
    if (PCArray[3] == true) {
      return true;
    } else {
      return false;
    }
  }

  public void moveIndexer(boolean newValue) {
    shiftIndexer = newValue; 
  }

  public boolean IndexerDone() {
    return shiftIndexer;
  }

  public boolean[] getPCArray() {
    return PCArray;
  }

  public void ResetEncoder() {
    mainEncoder.setPosition(0);
    //alternateEncoder.setPosition(0);
  }

  // Functions for compenstating based on indexer overshoot. We never tested these.
  public void CalculateOvershoot(double encoderVal, double desiredPosition) {
    overShoot = encoderVal - desiredPosition;
  }

  public double GetOvershoot() {
    return overShoot;
  }

  //TESTING FUNCTIONS 
  //For testing, this will be disabled later
  public double getdesiredSpeed() {
    return desiredSpeedNT.getDouble(0);
  }

}
