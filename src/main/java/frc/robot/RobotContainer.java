/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.cameraserver.CameraServer;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.cscore.UsbCamera;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

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
  private final Shooter roboShooter = new Shooter();
  private final Harvester roboHarvester = new Harvester(roboIndexer, roboShooter);
  private final Elevator roboElevator = new Elevator();

  // Define CameraServer
  public CameraServer RobotCamera;
  public UsbCamera frontRobotCamera;

  // private Command autoCommand = new AutonomousDrive(roboDT, 60);
  // private Command autoCommand = new AutonomousTurn(roboDT, 60, 90, true);
  // private Command autoShoot = new AutonomousShoot(roboShooter); // Stupid way to do this but a hot fix for testing
  SendableChooser<Command> mChooser = new SendableChooser<>();
  Trajectory barrelRun = new Trajectory();
  Trajectory slolam = new Trajectory();
  Trajectory bounce = new Trajectory();
  TrajectoryConfig config = 
      new TrajectoryConfig(AutoConstants.MAX_SPEED_MPS, AutoConstants.MAX_ACC_MPS)
      .setKinematics(AutoConstants.DRIVE_KINEMATICS);
  
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), // start at origin facing +X
        List.of( // pass interior way points, making 's' curve
            // new Translation2d(1, 1),
            // new Translation2d(2, -1)
        ),
        new Pose2d(3, 0, new Rotation2d(0)), // end 3m straight, facing forward
        config
    );
  
  private XboxController driver = new XboxController(OIConstants.DRIVER_CONTROLLER);
  private XboxController operator = new XboxController(OIConstants.OPERATOR_CONTROLLER);
  private Joystick joystick = new Joystick(1);
  private Joystick leftJoystick = new Joystick(2);

  private DriveConstants.driveModes driveMode = DriveConstants.driveModes.kCLGTA; // CHANGE ROBOT DRIVE TYPE HERE

  // Initialize Driver Buttons
  JoystickButton driverYButton = new JoystickButton(driver, Constants.Controller.XBOX.Y);
  JoystickButton driverStartButton = new JoystickButton(driver, Constants.Controller.XBOX.START);
  JoystickButton driverBackButton = new JoystickButton(driver, Constants.Controller.XBOX.BACK);
  JoystickButton driverAButton = new JoystickButton(driver, Constants.Controller.XBOX.A);
  JoystickButton driverBButton = new JoystickButton(driver, Constants.Controller.XBOX.B);

  // Initialize Operator Buttons
  JoystickButton operatorAButton = new JoystickButton(operator, Constants.Controller.XBOX.A);
  JoystickButton operatorBButton = new JoystickButton(operator, Constants.Controller.XBOX.B);
  JoystickButton operatorXButton = new JoystickButton(operator, Constants.Controller.XBOX.X);
  JoystickButton operatorYButton = new JoystickButton(operator, Constants.Controller.XBOX.Y);
  JoystickButton operatorLeftBumper = new JoystickButton(operator, Constants.Controller.XBOX.BUMPER.LEFT);
  JoystickButton operatorRightBumper = new JoystickButton(operator, Constants.Controller.XBOX.BUMPER.RIGHT);
  JoystickButton operatorStartButton = new JoystickButton(operator, Constants.Controller.XBOX.START);
  JoystickButton operatorBackButton = new JoystickButton(operator, Constants.Controller.XBOX.BACK);
  TriggerButton operatorLeftTrigger = new TriggerButton(operator, TriggerButton.TriggerSelection.LEFT);
  TriggerButton operatorRightTrigger = new TriggerButton(operator, TriggerButton.TriggerSelection.RIGHT);
  DPadButton operatorUpDpad = new DPadButton(operator, DPadButton.Direction.UP);
  DPadButton operatorDownDpad = new DPadButton(operator, DPadButton.Direction.DOWN);
  DPadButton operatorLeftDpad = new DPadButton(operator, DPadButton.Direction.LEFT);
  DPadButton operatorRightDpad = new DPadButton(operator, DPadButton.Direction.RIGHT);
  

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
  
    barrelRun = loadPathJSON(AutoConstants.BARREL_RUN_JSON);
    slolam = loadPathJSON(AutoConstants.SLOLAM_RUN_JSON);
    bounce = loadPathJSON(AutoConstants.BOUNCE_RUN_JSON);
    
    // Configure Auton Chooser
    mChooser.setDefaultOption("Barrel Run", pathAuto(barrelRun));
    mChooser.addOption("Slolam", pathAuto(slolam));
    mChooser.addOption("Example Auto", pathAuto(exampleTrajectory));
    mChooser.addOption("Do Nothing", new InstantCommand());
    mChooser.addOption("Bounce", pathAuto(bounce));
    SmartDashboard.putData("[Autonomous] Autonomous Chooser", mChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // DRIVER BUTTON ASSIGNMENTS
    // operatorStartButton.whenPressed(); // pause robot
    // operatorBackButton.whenPressed(); // emergency stop robot
    driverAButton.whenPressed(
      () -> roboElevator.engageRatchet()
    );
    driverBButton.whenPressed(
      () -> roboElevator.disengageRatchet()
    );

    // OPERATOR BUTTON ASSIGNMENTS
    operatorAButton.whenPressed(new MoveIndexer(roboIndexer, false)); // run intake state machine
    operatorBButton.whenPressed(new Shoot(roboIndexer, roboShooter)); // run shooting state machine
    operatorXButton.whenPressed(new Purge(roboShooter, roboIndexer, roboHarvester)); // purge powercells from robot
    operatorYButton.whenPressed(new MoveIndexer(roboIndexer, true)); // possibly rapid fire
    // operatorStartButton.whenPressed(); // pause robot
    // operatorBackButton.whenPressed(); // emergency stop robot

    operatorLeftBumper.whenHeld(new MoveElevator(roboElevator, false)); // commented until further testing is performed (ratchet wiring!!!)
    operatorRightBumper.whenHeld(new MoveElevator(roboElevator, true));

    operatorLeftTrigger.whenHeld(new MoveShooter(roboShooter, false)); // reverse shooter
    operatorRightTrigger.whenHeld(new MoveShooter(roboShooter, true)); // run shooter when held

    operatorUpDpad.whenPressed(new MoveIndexer(roboIndexer, true));
    operatorDownDpad.whenPressed(new MoveIndexer(roboIndexer, false));
    operatorLeftDpad.whenHeld(new MoveHarvester(roboHarvester, false));
    operatorRightDpad.whenHeld(new MoveHarvester(roboHarvester, true));

  }

  private void configureDriveMode() {

    // Switch statement for drive mode, drive mode is set above in member variables
    SmartDashboard.putString("[Drivetrain] Drive Mode", driveMode.toString());

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

      case kCLXboxArcade:
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
              (driver.getRawAxis(Controller.XBOX.TRIGGER.RIGHT) - driver.getRawAxis(Controller.XBOX.TRIGGER.LEFT))),
            roboDT
          )
        );
        break;

      // case kCLFlightArcade:
      //   roboDT.setDefaultCommand(
      //     new RunCommand(
      //       () -> roboDT.closedLoopArcadeDrive(
      //         leftJoystick.getZ(),
      //         -leftJoystick.getY()),
      //       roboDT
      //     )
      //   );
      //   break;

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
  public Command getAutonomousCommand() {
    return mChooser.getSelected();
  }

  public Command pathAuto(Trajectory trajectory){
  
    BiConsumer<Double, Double> putWheelSpeeds = 
        (x,y) -> {
              roboDT.setWheelSpeeds(x,y);
              // SmartDashboard.putNumber("[Autonomous] Ramsete Left Wheel Speed", x);
              // SmartDashboard.putNumber("[Autonomous] Ramsete Right Wheel Speed", y);
          };
  
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        roboDT::getPose, 
        new RamseteController(AutoConstants.RAMSETE_B, AutoConstants.RAMSETE_ZETA),
        AutoConstants.DRIVE_KINEMATICS,
        putWheelSpeeds,
        roboDT
    );

    // ramseteCommand.addRequirements(roboDT); // this might not be necessary or break things
  
    return ramseteCommand.beforeStarting(() -> roboDT.resetEncoders()).andThen(() -> roboDT.stopDT());
  }

  public Trajectory loadPathJSON(String file_path){
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(file_path);
      SmartDashboard.putString("[Autonomous] Trajectory Path", trajectoryPath.toString());
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex){
      DriverStation.reportError("Unable to open trajectory: " + AutoConstants.BARREL_RUN_JSON, ex.getStackTrace());
    }
    return trajectory;
  }

  public XboxController getDriverController() {
    return this.driver;
  }

  public Shooter getShooter() {
    return this.roboShooter;
  }


  public Elevator getElevator() {
    return this.roboElevator;
  }
  
  public AHRS getNavx(){
    return roboDT.getNavx();
  }
}
