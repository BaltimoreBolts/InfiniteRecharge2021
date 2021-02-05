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


/* 
** PURPOSE: Elevator sub-system. 
** STATUS: Tested pretty well
*/

public class Elevator extends SubsystemBase {
  private CANSparkMax ElevatorGoofyMotor;
  private Relay elevatoRelay;
  CANEncoder elevatorEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    ElevatorGoofyMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_GOOFY,MotorType.kBrushless);
    ElevatorGoofyMotor.restoreFactoryDefaults();
    ElevatorGoofyMotor.setSmartCurrentLimit(40);
    ElevatorGoofyMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    elevatorEncoder = ElevatorGoofyMotor.getAlternateEncoder(kAltEncType, 
                        Constants.GenConstants.REV_ENCODER_CPR); // Changed this, to match indexer, same encoder?
    elevatoRelay = new Relay(2, Relay.Direction.kForward);
    ElevatorGoofyMotor.burnFlash();
  }
  
  public void setSpeed(double speed){
    ElevatorGoofyMotor.set(speed);
  }

  // engage ratched by having the relay off. 
  // This way when power is cut to the robot, it doesn't drop
  public void engageRatchet () {
    elevatoRelay.set(Value.kOff);
  }

  // Must turn relay on to disengage ratchet and allow elevator movement
  public void disengageRatchet() {
    elevatoRelay.set(Value.kOn);
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
