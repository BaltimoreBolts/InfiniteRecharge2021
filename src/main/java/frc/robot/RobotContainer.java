/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.FirePowerCell;
import frc.robot.commands.IndexerCaptain;
import frc.robot.commands.PowerCellSucker;
import frc.robot.commands.RapidFire;
import frc.robot.commands.ShootPowerCell;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.AutonomousShoot;
import frc.robot.commands.ElevatorGoUp;
import frc.robot.commands.ElevatorGoDown;
import frc.robot.commands.moveIndexer;
import frc.robot.commands.reverseIndexer;
import frc.robot.commands.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controller;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final DriveTrain roboDT = new DriveTrain();
  private final Indexer roboIndexer = new Indexer();
  private final Shooter roboShoot = new Shooter();
  private final Harvester roboHarvest = new Harvester(roboIndexer, roboShoot);
  private final Elevator roboElevator = new Elevator();
  
  // Define CameraServer
  public CameraServer RobotCamera;
  public UsbCamera frontRobotCamera;
 
  private Command autoCommand = new AutonomousDrive(roboDT, 18);
  private Command autoShoot = new AutonomousShoot(roboShoot); // Stupid way to do this but a hot fix for testing 
  private XboxController driver = new XboxController(OIConstants.DRIVER_CONTROLLER);
  private XboxController operator = new XboxController(OIConstants.OPERATOR_CONTROLLER);

  JoystickButton rightDriverBumper;
  JoystickButton leftDriverBumper;
  JoystickButton xDriverButton;
  JoystickButton aDriverButton;
  JoystickButton bDriverButton;

  JoystickButton aOperatorButton;
  JoystickButton bOperatorButton;
  JoystickButton xOperatorButton;
  JoystickButton yOperatorButton;
  JoystickButton rightOperatorBumper;
  JoystickButton leftOperatorBumper;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set default drive command
    // Negative in the Y direction makes robot go forward 2/6
    
    roboDT.setDefaultCommand(
      new RunCommand(() -> roboDT
        .arcadeDrive(driver.getRawAxis(Controller.XBOX.STICK.LEFT.X), 
        -driver.getRawAxis(Controller.XBOX.STICK.LEFT.Y)), roboDT));
        RobotCamera = CameraServer.getInstance();
    frontRobotCamera = RobotCamera.startAutomaticCapture(0);
    
    /** serverOne = CameraServer.getInstance();
	    //serverOne.startAutomaticCapture();
	    //serverOne.startAutomaticCapture(0);
	    camera = serverOne.startAutomaticCapture(0);
	    camera.setResolution(RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT);
	    camera.setBrightness(50);
	    camera.setExposureManual(50); **/

  

  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {  
    rightDriverBumper = new JoystickButton(driver, Constants.Controller.XBOX.BUMPER.RIGHT);
    //leftDriverBumper = new JoystickButton(driver, Constants.Controller.XBOX.BUMPER.LEFT);
    xDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.X);
    aDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.A);
    bDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.B);

    aOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.A);
    bOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.B);
    xOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.X);
    yOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.Y);
    rightOperatorBumper = new JoystickButton(operator, Constants.Controller.XBOX.BUMPER.RIGHT);
    leftOperatorBumper = new JoystickButton(operator, Constants.Controller.XBOX.BUMPER.LEFT);
    
    //rightDriverTrigger.whenPressed(new FirePowerCell(roboShoot, roboIndexer, roboHarvest)); // Triggers are axis but that's hard
    rightDriverBumper.whenPressed(new FirePowerCell(roboShoot, roboIndexer, roboHarvest));
    //leftDriverBumper.whenPressed(new RapidFire(roboIndexer));
    xDriverButton.whenPressed(new IndexerCaptain(roboIndexer));

    bOperatorButton.whenPressed(new PowerCellSucker(roboHarvest, -1.0, true)); // top (near indexer) sucks in 
    xOperatorButton.whenPressed(new PowerCellSucker(roboHarvest, 1.0, true)); // top (near indexer) pushes out 
    rightOperatorBumper.whenPressed(new PowerCellSucker(roboHarvest, 1.0, false)); // front pushes out 
    leftOperatorBumper.whenPressed(new PowerCellSucker(roboHarvest, -1.0, false)); // front sucks in 

    yOperatorButton.whenPressed(new ElevatorGoUp(roboElevator));
    aOperatorButton.whenPressed(new ElevatorGoDown(roboElevator));

    //Testing Indexer rotation
    aDriverButton.whenPressed(new moveIndexer(roboIndexer));
    bDriverButton.whenPressed(new reverseIndexer(roboIndexer));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(boolean value) {
    // An ExampleCommand will run in autonomous
    // Again really dumb way to do this but the SequentialCommandGroup was breaking our code 
    if (value) {
      return autoCommand;
    } else {
      return autoShoot;
    }
  }

  public XboxController GetDriverController() {
    return this.driver;
  }

  public Shooter GetShooter() {
    return this.roboShoot;
  }

}
