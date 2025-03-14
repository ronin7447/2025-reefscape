// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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

  public static final double MAX_SPEED  = Units.feetToMeters(14.5);  //RL modify speed from 14.5 to 7.0
  public static final double DECREASED_SPEED  = Units.feetToMeters(7.25);
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

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.3;   // RL modify from 0.1 to 0.3
    public static final double LEFT_Y_DEADBAND = 0.3;    // RL modify from 0.1 to 0.3
    public static final double RIGHT_X_DEADBAND = 0.3;   // RL modify from 0.1 to 0.3
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class ElevatorConstants {

    public static final int ELEVATOR_MOTORID = 25;
    public static final double ELEVATOR_UP_SPEED = 0.75; // Testing speed is 0.3 / -0.3
    public static final double ELEVATOR_DOWN_SPEED = -0.75;

    //public static final int TRUE_BOTTOM = 0;
    public static final int L1_HEIGHT = 45; //change 0 to true bottom
    public static final int L2_HEIGHT = 86;
    public static final int L3_HEIGHT = 147;
    public static final double TOLERANCE = 0.1;
  }

  public static final class ShooterConstants {

    public static final int SHOOTER_MOTORID = 60;
    public static final double SHOOTER_SPEED_HIGH = 0.25;
    public static final double SHOOTER_SPEED_LOW = 0.125;
    public static final double SHOOTER_REVERSE_SPEED = -0.1;

  }

  public static final class ClimbConstants {

    public static final int CLIMB_MOTORID = 18;
    public static final double CLIMB_SPEED = 0.8;
    public static final double CLIMB_REVERSE_SPEED = -0.8;

  }

  public static final class VisionConstants {

   public static final double TAG_TO_CAMERA_DIFF = 10.00;

  }

}
