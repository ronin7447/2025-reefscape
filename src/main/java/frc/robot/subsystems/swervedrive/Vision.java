package frc.robot.subsystems.swervedrive;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;


public class Vision {
    // Basic targeting data
    // private double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
    // private double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
    // private double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
    // private boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    // private double txnc = LimelightHelpers.getTXNC("");  // Horizontal offset from principal pixel/point to target in degrees
    // private double tync = LimelightHelpers.getTYNC("");  // Vertical  offset from principal pixel/point to target in degrees

    

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
        double kP = 0.3;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight-front") * kP;
        // System.out.println(tx);
        // System.out.println("TX");
        // System.out.println(LimelightHelpers.getTX("limelight-front"));

        // convert to radians per second for our drive method
        targetingAngularVelocity *= 0.25;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double[] limelight_range_proportional()
  {    
 

    double targetingAngle = LimelightHelpers.getTX("limelight-front"); // in degrees

    double txnc = LimelightHelpers.getTXNC("limelight-front");

  
    double theta = targetingAngle + txnc;

    double txRad = Math.toRadians(targetingAngle);
    double txncRad = Math.toRadians(txnc);
    double thetaRad = Math.toRadians(theta);
    

    // double lateralTranslation;

    // if (Math.abs(txRad) > 1e-6) {
    //     lateralTranslation = 10 * Math.sin(txncRad) / Math.sin(txRad);
    // } else {
    //     // If tx is nearly zero, we assume no lateral translation is needed.
    //     lateralTranslation = 0.0;
    // }


    // double ta = LimelightHelpers.getTA("limelight-front");
    // System.out.println(ta);

    // // test using TA (area of the apriltag to stop)
    // if (ta >= 30) {
    //   double[] translations = new double[2];
    //   translations[0] = 0;
    //   translations[1] = 0;
    //   return translations;
    // }


    // double targetingAngleRad = Math.toRadians(targetingAngle); // convert to radians

    double[] translations = new double[2];


    // translations[0] = lateralTranslation * Math.sin(thetaRad);
    // translations[1] = lateralTranslation * Math.cos(thetaRad);

    // Optional scaling, if needed
    translations[0] *= 0.25;  
    translations[1] *= 0.25;

    return translations;
    
  }

  public double getTX() {
    return LimelightHelpers.getTX("limelight-front");
  }

  public double getTA() {
    return LimelightHelpers.getTA("limelight-front");
  }

  public double[] getTranslation() {
    double tx = LimelightHelpers.getTX("limelight-front");
    double txRad = Math.toRadians(tx);
    double[] translation = new double[2];

    translation[0] = Math.cos(txRad);
    translation[1] = Math.sin(txRad);

    return translation;
  }
}