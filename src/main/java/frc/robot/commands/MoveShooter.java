/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants.ShooterControlState;
import frc.robot.subsystems.Shooter;
import java.lang.Math;

/*
 * PURPOSE: Shoot a power cell.
 * STATUS: Never tested and probably doesn't work.
 * Needed to get interface with raspi vision processing to work (in shooter subsystem)
 * Also velocity PID control was never completely worked through/still in testing
 */
public class MoveShooter extends CommandBase {
  Shooter roboShooter;
  double calculatedRPM = 8000;
  boolean direction = true; // default to shooting

  /**
   * Creates a new ShootPowerCell. Direction True = Fire 
   */
  public MoveShooter(Shooter shooter, boolean direction, double calculatedRPM) {
    this.roboShooter = shooter;
    this.direction = direction;
    this.calculatedRPM = calculatedRPM;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  public MoveShooter(Shooter shooter, boolean direction){
    this(shooter, direction, 8000); // without rpm param, run at 8k rpm
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Math.abs(this.calculatedRPM) < 10) { // if the rpm is ~ 0 (double comparison issue), dont spin up
      roboShooter.setShooterState(ShooterControlState.IDLE);
    } else {
      roboShooter.setShooterState(ShooterControlState.SPINUP); // Robot Shooter is now set to spin up
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // roboShooter.PIDTuner(); // for testing (pid tuning)
    roboShooter.setReadyToFire(true);
    if (direction) {
      roboShooter.setDesiredRPM(-calculatedRPM);
    } else {
      roboShooter.setDesiredRPM(calculatedRPM);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //roboShooter.setShooterState(ShooterControlState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // stop shooter is we're at desired speed or have been told to idle
    return roboShooter.atSpeed(calculatedRPM) || Math.abs(this.calculatedRPM) < 10;
  }
}
