package frc.robot.subsystems.swervedrive;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;


public class Vision {
    // Basic targeting data
    private double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    private double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    private double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    private boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    private double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
    private double tync = LimelightHelpers.getTYNC("");  // Vertical  offset from principal pixel/point to target in degrees

    

    public Vision() {
        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("limelight-front", 0);
    }
    public double limelight_aim_proportional()
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = 0.1;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight-front") * kP;
        // System.out.println(tx);
        // System.out.println("TX");
        // System.out.println(LimelightHelpers.getTX("limelight-front"));

        // convert to radians per second for our drive method
        targetingAngularVelocity *= 0.5;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double[] limelight_range_proportional()
  {    
    // VERSION 1

    // double kP = .5;
    // double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    // System.out.println("TY");
    // System.out.println(LimelightHelpers.getTY("limelight-front"));
    
    // was 0.1 for now for testing
    // targetingForwardSpeed *= 0.5;
    // targetingForwardSpeed *= -1.0;
    // System.out.println("Kaden says that the tfs is" + targetingForwardSpeed);

    // double[] speed = new double[2];

    // speed[0] = targetingForwardSpeed;
    // speed[1] = targetingForwardSpeed;

    // return speed;

    // VERSION 2

    // double txncVal = LimelightHelpers.getTXNC("limelight-front");
    // double tyncVal = LimelightHelpers.getTYNC("limelight-front");
    // System.out.println("Kaden says the TXNC and TYNC is: " + txncVal + ", " + tyncVal);

    // double kP = 2.0;
    // double elevationAngle = LimelightHelpers.getTY("limelight") * kP;
    // System.out.println("Jun says the TY is " + LimelightHelpers.getTY("limelight-front"));

    // double targetingForwardSpeed = Constants.VisionConstants.TAG_TO_CAMERA_DIFF / Math.tan(elevationAngle);

    // targetingForwardSpeed *= 0.075;

    // System.out.println("Daniel says that the tfs is" + targetingForwardSpeed);

    // System.out.println();

    // return targetingForwardSpeed;

    // VERSION 3

    // double forwardSpeed = 1.0;

    // double targetingAngle = LimelightHelpers.getTX("limelight-front");
    // int targetingAngle_int = (int) targetingAngle;

    // double[] translations = new double[2];

    // translations[0] = forwardSpeed * Math.cos(targetingAngle_int);
    // translations[1] = forwardSpeed * Math.sin(targetingAngle_int);

    // translations[0] *= 0.5;
    // translations[1] *= 0.5;

    // return translations;

    //GPT VERSION 4

    double forwardSpeed = 1.0;

    double targetingAngle = LimelightHelpers.getTX("limelight-front"); // in degrees
    double targetingAngleRad = Math.toRadians(targetingAngle); // convert to radians

    double[] translations = new double[2];

    translations[0] = forwardSpeed * Math.cos(targetingAngleRad);
    translations[1] = forwardSpeed * Math.sin(targetingAngleRad);

// Optional scaling, if needed
    translations[0] *= 0.5;
    translations[1] *= 0.5;

    return translations;


  }
}