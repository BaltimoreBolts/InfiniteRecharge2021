/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import java.lang.Math;
import java.util.List;
import java.util.function.BiConsumer;

/* 
** PURPOSE: This is the autonmous drive commmand. Drives robot 
** forward/backward until certain distance has been travelled
** STATUS: Tested and works (signs of encoders, motor speeds should all be correct)
*/ 
 
public class AutonomousFollowTrajectory extends CommandBase {
  /**
   * Creates a new Autonomous.
   */

  DriveTrain roboDT;
  public AutonomousFollowTrajectory(DriveTrain robotDT) {
    // Use addRequirements() here to declare subsystem dependencies.
    roboDT = robotDT;
     // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboDT);

    TrajectoryConfig config = 
    new TrajectoryConfig(AutoConstants.MAX_SPEED_MPS, AutoConstants.MAX_ACC_MPS)
    .setKinematics(AutoConstants.DRIVE_KINEMATICS);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), // start at origin facing +X
        List.of( // pass interior way points, making 's' curve
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        new Pose2d(3, 0, new Rotation2d(0)), // end 3m straight, facing forward
        config
    );

    BiConsumer<Double, Double> setWheelSpeeds = (x,y) -> roboDT.setWheelSpeeds(x,y);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        roboDT::getPose, 
        new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
        AutoConstants.DRIVE_KINEMATICS,
        setWheelSpeeds,
        roboDT
    );
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roboDT.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roboDT.stopDT();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
