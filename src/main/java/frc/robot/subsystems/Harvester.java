/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HarvesterConstants;
import frc.robot.commands.*;

/* 
** PURPOSE: Harvester subsystem 
** STATUS: Motor functions are good. Note correct directions are set in the commands, 
** not here (may or may not have been the best idea). TOF sensor stuff has not been tested
** or really completed coding
*/

public class Harvester extends SubsystemBase {
  private CANSparkMax harvesterMickeyMotor; // The front one dawg
  private CANSparkMax harvesterMinnieMotor; // The back one (closer to the indexer)
  //private DigitalInput LimitSwitch0;
  private Indexer roboIndexer;
  private Relay harvesterRelease;
  private Shooter roboShooter;
  private TimeOfFlight harvesterTOF;
  NetworkTableEntry desiredFrontSpeedNT, desiredBackSpeedNT;

  /**
   * Creates a new Harvester.
   */
  public Harvester(Indexer robotIndex, Shooter robotShooter) {
  
    roboIndexer = robotIndex;
    roboShooter = robotShooter;
    harvesterMickeyMotor = new CANSparkMax (HarvesterConstants.HARVESTER_MOTOR_MICKEY, MotorType.kBrushless);
    harvesterMinnieMotor = new CANSparkMax (HarvesterConstants.HARVESTER_MOTOR_MINNIE, MotorType.kBrushless);
    harvesterMickeyMotor.restoreFactoryDefaults();
    harvesterMinnieMotor.restoreFactoryDefaults();
    harvesterMickeyMotor.setSmartCurrentLimit(30);
    harvesterMinnieMotor.setSmartCurrentLimit(30);
    harvesterMickeyMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    harvesterMinnieMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
   
    // LimitSwitch0 = new DigitalInput(HarvesterConstants.HARVESTER_LIMIT_SWITCH);
    harvesterTOF = new TimeOfFlight(HarvesterConstants.HARVESTER_TOF);
    harvesterRelease = new Relay(0);

    harvesterRelease.set(Relay.Value.kOn);

    harvesterMickeyMotor.burnFlash();
    harvesterMinnieMotor.burnFlash();

    ShuffleboardTab harvesterTab = Shuffleboard.getTab("Harvester");
    desiredFrontSpeedNT = harvesterTab.add("Desired Front Speed = ", 0).getEntry();
    desiredBackSpeedNT = harvesterTab.add("Desired Back Speed = ", 0).getEntry();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Harvester TOF", this.getHarvesterTOF());
    SmartDashboard.putNumber("Harvester TOF Val", harvesterTOF.getRange());
    desiredFrontSpeedNT.getDouble(0);
    desiredBackSpeedNT.getDouble(0);

    // Idea here was, if we see a PC in front of us, automatically start ingesting if we have room
    if (this.getHarvesterTOF() == true){
        //new IndexerHarvestMayhem(roboIndexer, this, roboShooter);
    }
  }

  // Mickey is the front harvester motor
  public void setMickeySpeed(double speed){
    harvesterMickeyMotor.set(speed);
  }
  
  // Minnie is the back harvester motor
  public void setMinnieSpeed(double speed){
    harvesterMinnieMotor.set(speed);
  }

  //Return value of first position optical sensor
  public boolean getHarvesterTOF(){
    // return LimitSwitch0.get();
    // Because the ball is curved we want to stop when the center of the ball is in front of the sensor hence the range 
    return harvesterTOF.getRange() >= 15 && harvesterTOF.getRange() <= 25; 
  }

  // TESTING FUNCTIONS
  //For testing, this will be disabled later
  public double getFrontDesiredSpeed() {
    return desiredFrontSpeedNT.getDouble(0);
  }
  //For testing, this will be disabled later
  public double getBackDesiredSpeed() {
    return desiredBackSpeedNT.getDouble(0);
  }


}
