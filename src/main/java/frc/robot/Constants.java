/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.Math;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    //General constants
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
    }

     // To import this elsewhere use import import frc.robot.Constants.OIConstants;
     public static final class OIConstants {
        //These need to be public within the class so they are accessible
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;
    }

    public static final class DriveConstants {
        public static final int LEFT_DRIVE_MOTOR1 = 1;
        public static final int LEFT_DRIVE_MOTOR2 = 2;
        public static final int RIGHT_DRIVE_MOTOR1 = 3;
        public static final int RIGHT_DRIVE_MOTOR2 = 4;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_CHIP = 9;
        public static final int SHOOTER_MOTOR_DALE = 10;
    }
    public static final class HarvesterConstants {
        public static final int HARVESTER_MOTOR_MICKEY = 6;
        public static final int HARVESTER_MOTOR_MINNIE = 5;
        public static final int HARVESTER_LIMIT_SWITCH = 0;
        public static final int HARVESTER_TOF = 11;

        // tof is the time of flight sensor
    

    }
    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR_DONALD = 7;
        public static final int INDEXER_LIMIT_SWITCH1 = 1;
        public static final int INDEXER_LIMIT_SWITCH2 = 2;
        public static final int INDEXER_LIMIT_SWITCH3 = 3;
        public static final int INDEXER_LIMIT_SWITCH4 = 4;
        public static final int INDEXER_LIMIT_SWITCH5 = 5;
        public static final int INDEXER_TOF = 12;
    } 

    public static final class ElevatorConstants {
        public static final int ELEVATOR_MOTOR_GOOFY = 8;
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
}
