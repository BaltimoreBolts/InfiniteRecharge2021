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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double kP = 0.1; // 2e-5 initial test value 
  private double kI = 1e-4; // 0 initial test value 
  private double kD = 1; // 0 initial test value 
  private double kFF = 0.000165; // 0.000165 initial test value

  //private DigitalInput OpticalSensor;
  private TimeOfFlight IndexerTOF;
  private CANEncoder alternateEncoder;
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
    alternateEncoder = IndexerDonaldMotor.getAlternateEncoder(kAltEncType, 
                        Constants.GenConstants.REV_ENCODER_CPR);
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
    indexerPID.setFeedbackDevice(alternateEncoder);

    indexerPID.setP(kP);
    indexerPID.setI(kI);
    indexerPID.setD(kD); 
    indexerPID.setOutputRange(-1, 1);  
    IndexerDonaldMotor.getPIDController();

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
    SmartDashboard.putNumber("Current pVal = ", kP);
    SmartDashboard.putNumber("Current iVal = ", kI);
    SmartDashboard.putNumber("Current dVal = ", kD);

    this.ResetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    UpdateDashboard();
  
    SmartDashboard.putBoolean("Indexer TOF", this.getP0());
    SmartDashboard.putNumber("Indexer TOF Val", IndexerTOF.getRange());
    SmartDashboard.putNumber("Indexer Encoder", this.getEncoderValue());
    SmartDashboard.putBooleanArray("Indexer Array", PCArray);
    indexerSpeed = SmartDashboard.getNumber("Indexer Speed", 0);
    SmartDashboard.putNumber("Indexer Overshoot", overShoot);

    //IndexerDonaldMotor.set(indexerSpeed);
  }

  public int degreeToCounts(double degrees, int CPR ){
    int Counts = 0;
    Counts = (int)Math.ceil(CPR * degrees/360.0);
    return Counts;

  }
  
  /*Publish values we want to look at to dashboard */
  public void UpdateDashboard() {
    currentRotationNT.setDouble(alternateEncoder.getPosition());
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

  public boolean MoveToPosition(double desiredPosition) {
    // Pass in the position you want to be in, return true / false if you're there? 
    // this.indexerPID.setReference(desiredPosition, ControlType.kPosition); // well this breaks things but is PID control.......
    
    return this.getEncoderValue() >= desiredPosition;
  }

  public double getEncoderValue(){
    return alternateEncoder.getPosition();
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
    alternateEncoder.setPosition(0);
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
