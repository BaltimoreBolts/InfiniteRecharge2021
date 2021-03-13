/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;


/*
** PURPOSE: Elevator sub-system.
** STATUS: Tested pretty well
*/

public class Elevator extends SubsystemBase {
  private CANSparkMax mElevatorMotor;
  private Servo mElevatorRatchet;
  CANEncoder mElevatorEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    mElevatorMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR,MotorType.kBrushless);
    mElevatorMotor.restoreFactoryDefaults();
    mElevatorMotor.setSmartCurrentLimit(40);
    mElevatorMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    mElevatorEncoder = mElevatorMotor.getAlternateEncoder(kAltEncType,
                        Constants.GenConstants.REV_ENCODER_CPR); // Changed this, to match indexer, same encoder?
    mElevatorRatchet = new Servo(9);
    mElevatorMotor.burnFlash();
  }

  @Override
  public void periodic() {
    updateSmartdashboard();
  }

  public void setSpeed(double speed){
    mElevatorMotor.set(speed);
  }

  // disengage ratched by turning servo to the right.
  public void disengageRatchet () {
    mElevatorRatchet.set(100.0/180.0);
    long start_time = System.nanoTime();
    while ((System.nanoTime() - start_time) < 1e9) {
    };
  }

  // Move servo to the left to engage the ratchet
  public void engageRatchet() {
    mElevatorRatchet.set(90.0/180.0);
    long start_time = System.nanoTime();
    while ((System.nanoTime() - start_time) < 1e9) {
    };
  }

  // Encoder was reading negative (I think), which is why we return the negative value
  public double getElevatorEncoder() {
    return -mElevatorEncoder.getPosition();
  }

  private void updateSmartdashboard(){
    SmartDashboard.putNumber("Elevator pos", -mElevatorEncoder.getPosition());
  }
}
