/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FirePowerCell;
import frc.robot.commands.IndexerCaptain;
import frc.robot.commands.PowerCellSucker;
import frc.robot.commands.RapidFire;
import frc.robot.commands.ShootPowerCell;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.AutonomousShoot;
import frc.robot.commands.ElevatorGoUp;
import frc.robot.commands.ElevatorGoDown;
import frc.robot.commands.MoveIndexer;
import frc.robot.commands.ReverseIndexer;
import frc.robot.commands.HarvesterIn;
import frc.robot.commands.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private Joystick joystick = new Joystick(1);
  private Joystick leftJoystick = new Joystick(2);

  private DriveConstants.driveModes driveMode = DriveConstants.driveModes.kCLGTA; // CHANGE ROBOT DRIVE TYPE HERE

  // Initialize Driver Buttons
  JoystickButton yDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.Y);
  JoystickButton startDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.START);
  JoystickButton backDriverButton = new JoystickButton(driver, Constants.Controller.XBOX.BACK);

  // Initialize Operator Buttons
  JoystickButton aOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.A);
  JoystickButton bOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.B);
  JoystickButton xOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.X);
  JoystickButton yOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.Y);
  JoystickButton leftOperatorBumper = new JoystickButton(operator, Constants.Controller.XBOX.BUMPER.LEFT);
  JoystickButton rightOperatorBumper = new JoystickButton(operator, Constants.Controller.XBOX.BUMPER.RIGHT);
  JoystickButton leftOperatorTrigger = new JoystickButton(operator, Constants.Controller.XBOX.TRIGGER.LEFT);
  JoystickButton rightOperatorTrigger = new JoystickButton(operator, Constants.Controller.XBOX.TRIGGER.RIGHT);
  JoystickButton leftOperatorDpad = new JoystickButton(operator, Constants.Controller.XBOX.DPAD.LEFT);
  JoystickButton rightOperatorDpad = new JoystickButton(operator, Constants.Controller.XBOX.DPAD.RIGHT);
  JoystickButton upOperatorDpad = new JoystickButton(operator, Constants.Controller.XBOX.DPAD.UP);
  JoystickButton downOperatorDpad = new JoystickButton(operator, Constants.Controller.XBOX.DPAD.DOWN);
  JoystickButton startOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.START);
  JoystickButton backOperatorButton = new JoystickButton(operator, Constants.Controller.XBOX.BACK);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure the control scheme for the drive train
    configureDriveMode();

    // Configure the camera
    configureCamera();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVER BUTTON ASSIGNMENTS

    // OPERATOR BUTTON ASSIGNMENTS
    // aOperatorButton.whenPressed(); // run intake state machine
    // bOperatorButton.whenPressed(); // run shooting state machine
    bOperatorButton.whenHeld(new PowerCellSucker(roboHarvest, -1.0, true), true); // top (near indexer) sucks in
    xOperatorButton.whenHeld(new PowerCellSucker(roboHarvest, 1.0, true), true); // top (near indexer) pushes out
    rightOperatorBumper.whenHeld(new PowerCellSucker(roboHarvest, 1.0, false), true); // front pushes out
    leftOperatorBumper.whenHeld(new PowerCellSucker(roboHarvest, -1.0, false), true); // front sucks in

    leftOperatorBumper.whenPressed(new ElevatorGoDown(roboElevator));
    rightOperatorBumper.whenPressed(new ElevatorGoUp(roboElevator));
  }

  private void configureDriveMode() {

    // Switch statement for drive mode, drive mode is set above in member variables
    SmartDashboard.putString("Drive Mode", driveMode.toString());

    switch (driveMode) {
      case kArcade:
        roboDT.setDefaultCommand(
          new RunCommand(
            () -> roboDT.arcadeDrive(
              driver.getRawAxis(Controller.XBOX.STICK.LEFT.X),
              -driver.getRawAxis(Controller.XBOX.STICK.LEFT.Y)),
            roboDT
          )
        );
        break;

      case kCLArcade:
        roboDT.setDefaultCommand(
          new RunCommand(
            () -> roboDT.closedLoopArcadeDrive(
              driver.getRawAxis(Controller.XBOX.STICK.LEFT.X),
              -driver.getRawAxis(Controller.XBOX.STICK.LEFT.Y)),
            roboDT
          )
        );
        break;

      case kCLSplitArcade:
        roboDT.setDefaultCommand(
          new RunCommand(
            () -> roboDT.closedLoopArcadeDrive(
              joystick.getX(),
              -leftJoystick.getY()),
            roboDT
          )
        );
        break;

      case kCLGTA:
        roboDT.setDefaultCommand(
          new RunCommand(
            () -> roboDT.closedLoopArcadeDrive(
              driver.getRawAxis(Controller.XBOX.STICK.LEFT.X)*0.55,
              -(driver.getRawAxis(Controller.XBOX.TRIGGER.LEFT) - driver.getRawAxis(Controller.XBOX.TRIGGER.RIGHT))*0.7),
            roboDT
          )
        );
        break;

      default:
        System.out.println("Drive Mode is not a valid implemented option. Defaulting to GTA...");
        roboDT.setDefaultCommand(
          new RunCommand(
            () -> roboDT.closedLoopArcadeDrive(
              driver.getRawAxis(Controller.XBOX.STICK.LEFT.X),
              -(driver.getRawAxis(Controller.XBOX.TRIGGER.LEFT) - driver.getRawAxis(Controller.XBOX.TRIGGER.RIGHT))),
            roboDT
          )
        );
    }
  }

  private void configureCamera() {
    RobotCamera = CameraServer.getInstance();
    frontRobotCamera = RobotCamera.startAutomaticCapture(0);

    // Camera code
    // serverOne = CameraServer.getInstance();
    // // serverOne.startAutomaticCapture();
    // // serverOne.startAutomaticCapture(0);
    // camera = serverOne.startAutomaticCapture(0);
    // camera.setResolution(RobotMap.IMG_WIDTH, RobotMap.IMG_HEIGHT);
    // camera.setBrightness(50);
    // camera.setExposureManual(50);
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
