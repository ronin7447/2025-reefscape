package frc.robot.subsystems.swervedrive;

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
        LimelightHelpers.setPipelineIndex("", 0);
    }
    public double limelight_aim_proportional()
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate around.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= 0.1;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -0.1;

        return targetingAngularVelocity;
    }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    // 0.1 for now for testing
    targetingForwardSpeed *= 0.1;
    targetingForwardSpeed *= 0.1;
    return targetingForwardSpeed;
  }
}