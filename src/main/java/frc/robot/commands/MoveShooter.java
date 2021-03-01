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
   * Creates a new ShootPowerCell.
   */
  public MoveShooter(Shooter shooter, boolean direction) {
    this.roboShooter = shooter;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // calculatedRPM = roboShooter.getNeededRPM(); // getRPM calculation from camera values
    roboShooter.setShooterState(ShooterControlState.SPINUP); // Robot Shooter is now set to spin up
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
    roboShooter.setShooterState(ShooterControlState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return roboShooter.SetShooterSpeed(calculatedRPM);
    // return roboShooter.AtSpeed(calculatedRPM);
    return false;
  }
}
