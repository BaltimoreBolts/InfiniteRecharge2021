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
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;


/*
** PURPOSE: Elevator sub-system.
** STATUS: Tested pretty well
*/

public class Elevator extends SubsystemBase {
  private CANSparkMax ElevatorGoofyMotor;
  private Servo elevatorRatchet;
  CANEncoder elevatorEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    ElevatorGoofyMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR,MotorType.kBrushless);
    ElevatorGoofyMotor.restoreFactoryDefaults();
    ElevatorGoofyMotor.setSmartCurrentLimit(40);
    ElevatorGoofyMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    elevatorEncoder = ElevatorGoofyMotor.getAlternateEncoder(kAltEncType,
                        Constants.GenConstants.REV_ENCODER_CPR); // Changed this, to match indexer, same encoder?
    elevatorRatchet = new Servo(8);
    ElevatorGoofyMotor.burnFlash();
  }

  public void setSpeed(double speed){
    ElevatorGoofyMotor.set(speed);
  }

  // engage ratched by turning servo to the right.
  public void engageRatchet () {
    elevatorRatchet.set(1);
    long start_time = System.nanoTime();
    while ((System.nanoTime() - start_time) < 1e9) {
    };
  }

  // Move servo to the left to engage disengage the ratchet
  public void disengageRatchet() {
    elevatorRatchet.set(0);
    long start_time = System.nanoTime();
    while ((System.nanoTime() - start_time) < 1e9) {
    };
  }

  // Encoder was reading negative (I think), which is why we return the negative value
  public double getElevatorEncoder() {
    return -elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator pos", -elevatorEncoder.getPosition());
  }
}
