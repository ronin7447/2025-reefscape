// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
// Hi says daniel
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static boolean isReducedSpeed = false; // Flag to toggle reduced speed
  public static final double MAX_SPEED_NORMAL = Units.feetToMeters(14.5); // Normal max speed
  public static final double MAX_SPEED_REDUCED = Units.feetToMeters(10); // Reduced max speed

  public static double getMaxSpeed() {
    return isReducedSpeed ? MAX_SPEED_REDUCED : MAX_SPEED_NORMAL;
  }

  public static final double DECREASED_SPEED  = 0.5; // Distance units
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class LimelightConstants {

    public static final String FRONTLL = "limelight-front";
    public static final String BACKLL = "limelight-back";

    public static final int[] CORAL_IDS = {};
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND         = 0.3;   // RL modify from 0.1 to 0.3
    public static final double LEFT_Y_DEADBAND  = 0.3;    // RL modify from 0.1 to 0.3
    public static final double RIGHT_X_DEADBAND = 0.3;   // RL modify from 0.1 to 0.3
    public static final double TURN_CONSTANT    = 6;
    
    // Secondary Operator Joystick
    // public static final int JOYSTICK_UP = 0;
    // public static final int JOYSTICK_RIGHT = 90;
    // public static final int JOYSTICK_DOWN = 180;
    // public static final int JOYSTICK_LEFT = 270;

    public static final int AXIS_X = 0;
    public static final int AXIS_Y = 1;

    // Secondary Operator Buttons
    public static final int BUTTON_1 = 1;
    public static final int BUTTON_2 = 2;
    public static final int BUTTON_3 = 3;
    public static final int BUTTON_4 = 4;
    public static final int BUTTON_5 = 5;
    public static final int BUTTON_6 = 6;
    public static final int BUTTON_7 = 7;
    public static final int BUTTON_8 = 8;
    public static final int BUTTON_9 = 9;
    public static final int BUTTON_10 = 10;
    
    
  }

  public static final class ElevatorConstants {

    public static final double BASE_SPEED = 0.05; // For elevator new function 3/21/25

    public static final int ELEVATOR_MOTORID = 25;
    public static final double ELEVATOR_UP_SPEED = 0.15; // Testing speed is 0.3 / -0.3
    public static final double ELEVATOR_DOWN_SPEED = -0.15;
    public static final double ELEVATOR_SLOW_SPEED = 0.1;

    //public static final int TRUE_BOTTOM = 0;
    public static final int L1_HEIGHT = 46; //change 0 to true bottom
    public static final int L2_HEIGHT = 94;
    public static final int L3_HEIGHT = 151; // new one is 161
    public static final double TOLERANCE = 0.1;

    // New Elevator Maths

    // Distances from 0 to L1, L2, and L3 perfect heights
    public static final int[] distances = {42, 86, 147};
    public static final int[] distanceToEncoder = {9, 2, 8}; // KADEN PLACEHOLDERS WE NEED TO FIND THESE 3/27/25
    

    // ABS Elevator heights
    // I think 0.0 is bottom (0)
    public static final double L1_ABS = 0.755859375;
    public static final double L2_ABS = 1.7958984375;
    public static final double L3_ABS = 3.2839355468749996;

  }

  public static final class ShooterConstants {

    public static final int SHOOTER_MOTORID = 60;
    public static final double SHOOTER_SPEED_HIGH = 0.35;
    public static final double SHOOTER_SPEED_LOW = 0.125;
    public static final double SHOOTER_REVERSE_SPEED = -0.15; // NOW USED FOR ALGAE REMOVAL

  }

  public static final class ClimbConstants {

    public static final int CLIMB_MOTORID = 18;
    // public static final int CLIMB_ENCODERID = 0; // replace with the correct encoder ID
    public static final double CLIMB_SPEED = 1.1;
    public static final double CLIMB_REVERSE_SPEED = -1.1;

    // public static final double CLIMB_OUT_ENCODER_POSITION = 0.0; // replace with the correct encoder position
    // public static final double CLIMB_IN_ENCODER_POSITION = 0.0; // replace with the correct encoder position

  }

  public static final class VisionConstants {

    //public static final double TAG_TO_CAMERA_DIFF = 10.00;
    public static final double X_REEF_ALIGNMENT_P = 3.35;
    public static final double Y_REEF_ALIGNMENT_P = 2.875;
    public static final double ROT_REEF_ALIGNMENT_P = 0.012;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.05;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.18;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.05;


    // Offsets for command
    public static final double Y_SETPOINT_REEF_ALIGNMENT_CENTER = 0.0;  // Horizontal pose
    public static final double Y_SETPOINT_REEF_ALIGNMENT_LEFT = -0.223;
    public static final double Y_SETPOINT_REEF_ALIGNMENT_RIGHT = 0.223;




    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.05;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;

    //Offsets for moving a little
    public static final double MOVEALITTLE_LEFT = 1;
    public static final double MOVEALITTLE_RIGHT = -1;
    public static final double DISTANCE_TOLERANCE = 0.05;   // Tolerance in meters
    public static final double ANGLE_TOLERANCE = 0.1;       // Tolerance in radians


  }

  public static final class AlgaeConstants {
    
    public static final int ALGAE_MOTORID = 16;
    public static final double ALGAE_SPEED = -0.3;
    public static final double ALGAE_REVERSE_SPEED = 0.3;

  }

}
