/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;


//cringe - i am alive dylan

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class Autonomous extends SequentialCommandGroup {

  /**
   * Creates a new Autonomous.
   */
  public Autonomous (DriveTrain drive, Shooter shoot) {
    
    //i was here

    //Sets shooter speed, shoots all power cells, sets shooter speed to 0, and moves drive train

    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    //double inchesToTravel = 18;
    //new AutonomousShoot();
    //new AutonomousDrive(drive,inchesToTravel);
    
    super(
      new AutonomousShoot(shoot),
      new AutonomousDrive(drive,18)
    );
  }
}
