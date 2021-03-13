/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.Math;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * PURPOSE: Constants file
 */
public final class Constants {
    // General constants
    public static final class GenConstants {
        public static final int REV_ENCODER_CPR = 8192;
        public static final double DEG2RAD = Math.PI/180.0;
        public static final double G_FT_PER_SEC2 = 32.2; //ft per second squared
        public static final double M_TO_FEET = 3.28084;
        public static final double INNER_PORT_HEIGHT_FT = 8.17;
        public static final double SHOOTER_HEIGHT_FT = 1;
        public static final double SHOOTER_ANGLE_DEG = 0;
        public static final double COS_ANGLE = Math.cos(SHOOTER_ANGLE_DEG*DEG2RAD);
        public static final double TAN_ANGLE = Math.tan(SHOOTER_ANGLE_DEG*DEG2RAD);
        public static final double IN_TO_M = 39.3701;
    }

     // To import this elsewhere use import frc.robot.Constants.OIConstants;
     public static final class OIConstants {
        // These need to be public within the class so they are accessible
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 3;
    }

    public static final class DriveConstants {
        public static final int LEFT_DRIVE_MOTOR1 = 1;
        public static final int LEFT_DRIVE_MOTOR2 = 2;
        public static final int RIGHT_DRIVE_MOTOR1 = 3;
        public static final int RIGHT_DRIVE_MOTOR2 = 4;

        public static final double WHEEL_CIRCUMFERENCE = 6 * Math.PI;
        public static final double GEARBOX_RATIO = 12.05; 
        public static final double MAX_RPM = 5676; // REV Neo free sped = 5676 rpm, gearbox = 12.05:1;
        public static final double TRACK = 24; // width from center of left wheel to right wheel in inches

        public static final double kP = 0.0001;  
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kFF = 1.0 / MAX_RPM; 

        public static final double MAX_VEL = MAX_RPM;    // rpm
        public static final double MIN_VEL = 0;      // rpm
        public static final double MAX_ACC = 5000;    // rpm/s
        public static final double ALLOWED_ERROR = 0; // rpm? 

        public static enum driveModes {
            kCLGTA("Closed Loop GTA"), // Triggers on XBOX controller + left axis, closed loop
            kCLXboxArcade("Closed Loop Arcade"), // left axis on XBOX controller, closed loop
            kCLSplitArcade("Closed Loop Split Arcade"), // two joysticks closed loop
            // kCLFlightArcade("Closed Loop Single Arcade"), // TODO: single joystick closed loop
            kArcade("Arcade"); // single joystick, open loop

            private String name;
            private driveModes(String name) {
                this.name = name;
            }

            @Override
            public String toString() {
                return name;
            }
        };
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_LEFT = 9;
        public static final int SHOOTER_MOTOR_RIGHT = 10;
        public static final int PV_RING_LIGHT = 0;
        public static enum ShooterControlState {
            IDLE("Idle"),
            SPINUP("Spin Up"),
            HOLDWHENREADY("Hold When Ready"),
            HOLD("Hold");

            private String name;
            private ShooterControlState(String name) {
                this.name = name;
            }

            @Override
            public String toString() {
                return name;
            }
        }
        public static final int kFFCircularBufferSize = 20;
        public static final double SHOOTER_FLYWHEEL_DIAMETER = 5; // TODO set this correctly with appropriate units
    }

    public static final class PowerCellConstants {
    }

    public static final class HarvesterConstants {
        public static final int HARVESTER_FRONT_MOTOR = 6;
        public static final int HARVESTER_BACK_MOTOR = 5;
        public static final int HARVESTER_LIMIT_SWITCH = 0;
        // tof is the time of flight sensor
        public static final int HARVESTER_TOF = 11;
    }

    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR = 7;
        public static final int INDEXER_LIMIT_SWITCH1 = 1;
        public static final int INDEXER_LIMIT_SWITCH2 = 2;
        public static final int INDEXER_LIMIT_SWITCH3 = 3;
        public static final int INDEXER_LIMIT_SWITCH4 = 4;
        public static final int INDEXER_LIMIT_SWITCH5 = 5;
        public static final int INDEXER_TOF = 12;

        public static final double kP = 0.07; // 2e-5 initial test value
        public static final double kI = 0.03; // 0 initial test value
        public static final double kD = 0.0; // had success 0.03 but need to check
        public static final double kFF = 0;

        public static final double kMaxPIDduration = 1e9; // 1 second in nano seconds

    }

    public static final class ElevatorConstants {
        public static final int ELEVATOR_MOTOR = 8;
    }

    // Add controller constant
    public static final class Controller {
        public static final class XBOX {
            public static final int A = 1;
            public static final int B = 2;
            public static final int X = 3;
            public static final int Y = 4;
            public static final int BACK = 7;
            public static final int START = 8;

            public static final class BUMPER {
                public static final int LEFT = 5;
                public static final int RIGHT = 6;
            }

            // handy boolean conversion
            // if (controller.getPOV() == Controller.DPAD.UP) == true
            public static final class DPAD {
                public static final int UP = 0;
                public static final int UP_RIGHT = 45;
                public static final int RIGHT = 12;
                public static final int DOWN_RIGHT = 135;
                public static final int DOWN = 180;
                public static final int DOWN_LEFT = 225;
                public static final int LEFT = 270;
                public static final int UP_LEFT = 315;
            }

            public static final class TRIGGER {
                public static final int LEFT = 2;
                public static final int RIGHT = 3;
            }

            public static final class STICK {
                public static final class LEFT {
                    public static final int X = 0;
                    public static final int Y = 1;
                }

                public class RIGHT {
                    public static final int X = 4;
                    public static final int Y = 5;
                }
            }
        }
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_MPS = 3; // meters per second
        public static final double MAX_ACC_MPS = 6; // meters per second^2
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(DriveConstants.TRACK * GenConstants.IN_TO_M); // convert to meters
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;
        public static final SerialPort.Port NAVX_PORT = SerialPort.Port.kUSB; // TODO kUSB vs kUSB1 vs kUSB2?
    }
}
