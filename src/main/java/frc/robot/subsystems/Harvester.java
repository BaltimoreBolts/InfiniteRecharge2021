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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HarvesterConstants;

/*
 * PURPOSE: Harvester subsystem
 * STATUS: Motor functions are good. Note correct directions are set in the commands,
 * not here (may or may not have been the best idea). TOF sensor stuff has not been tested
 * or really completed coding
 */

public class Harvester extends SubsystemBase {
  private CANSparkMax mHarvesterFrontMotor;
  private CANSparkMax mHarvesterBackMotor; // Closer to the indexer
  // private DigitalInput LimitSwitch0;
  private TimeOfFlight mHarvesterTOF;
  NetworkTableEntry mDesiredFrontSpeedNT, mDesiredBackSpeedNT;

  /**
   * Creates a new Harvester.
   */
  public Harvester(Indexer robotIndexer, Shooter robotShooter) {

    mHarvesterFrontMotor = new CANSparkMax(HarvesterConstants.HARVESTER_FRONT_MOTOR, MotorType.kBrushless);
    mHarvesterBackMotor = new CANSparkMax(HarvesterConstants.HARVESTER_BACK_MOTOR, MotorType.kBrushless);
    mHarvesterFrontMotor.restoreFactoryDefaults();
    mHarvesterBackMotor.restoreFactoryDefaults();
    mHarvesterFrontMotor.setSmartCurrentLimit(30);
    mHarvesterBackMotor.setSmartCurrentLimit(30);
    mHarvesterFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mHarvesterBackMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mHarvesterFrontMotor.burnFlash();
    mHarvesterBackMotor.burnFlash();

    // LimitSwitch0 = new DigitalInput(HarvesterConstants.HARVESTER_LIMIT_SWITCH);
    mHarvesterTOF = new TimeOfFlight(HarvesterConstants.HARVESTER_TOF);

    ShuffleboardTab harvesterTab = Shuffleboard.getTab("Harvester");
    mDesiredFrontSpeedNT = harvesterTab.add("Desired Front Speed = ", 0).getEntry();
    mDesiredBackSpeedNT = harvesterTab.add("Desired Back Speed = ", 0).getEntry();
  }

  @Override
  public void periodic() {

    mDesiredFrontSpeedNT.getDouble(0);
    mDesiredBackSpeedNT.getDouble(0);
    updateSmartdashboard();

  }

  public void setHarvesterFrontSpeed(double speed){
    mHarvesterFrontMotor.set(speed);
  }

  public void setHarvesterBackSpeed(double speed){
    mHarvesterBackMotor.set(speed);
  }

  // Return value of first position optical sensor
  public boolean getHarvesterTOF(){
    // return LimitSwitch0.get();
    // Because the ball is curved we want to stop when the center of the ball is in front of the sensor hence the range
    return mHarvesterTOF.getRange() >= 15 && mHarvesterTOF.getRange() <= 25;
    // we want to replace with with the indexer tof? range around <= 100 i think
  }

  private void updateSmartdashboard() {
    SmartDashboard.putBoolean("[Harvester] Harvester TOF", this.getHarvesterTOF());
    SmartDashboard.putNumber("[Harvester] Harvester TOF Val", mHarvesterTOF.getRange());
  }
}
