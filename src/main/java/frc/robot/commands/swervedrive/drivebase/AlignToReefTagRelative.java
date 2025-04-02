// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private int side;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = 18;
  private boolean done;

  public AlignToReefTagRelative(int side, SwerveSubsystem drivebase) {
    xController = new PIDController(Constants.VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0.04);  // Vertical movement
    yController = new PIDController(Constants.VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.04);  // Horitontal movement
    rotController = new PIDController(Constants.VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.side = side;
    this.drivebase = drivebase;

    done = false;

    addRequirements(drivebase);
    
  }

  public boolean getDone() {
    return done;
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);


    if (side == 0) {
      yController.setSetpoint(Constants.VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_LEFT);
    } else if (side == 1) {
      yController.setSetpoint(Constants.VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_CENTER);
    } else if (side == 2) {
      yController.setSetpoint(Constants.VisionConstants.Y_SETPOINT_REEF_ALIGNMENT_RIGHT);
    }

    // yController.setSetpoint(isRightScore ? Constants.VisionConstants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.VisionConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-front");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-front") && LimelightHelpers.getFiducialID("limelight-front") == tagID) {
      System.out.println("sees smth");

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-front");

      double xSpeed = xController.calculate(postions[2]) / 2;
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]) / 2;
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false, true);

    
    } else {
      drivebase.drive(new Translation2d(), 0, false);
      System.out.println("no sees smth");
      done = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.VisionConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.VisionConstants.POSE_VALIDATION_TIME);
  }
}