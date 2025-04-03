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
import frc.robot.commands.swervedrive.drivebase.SetAprilTagFilter;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private int side;
  private SwerveSubsystem drivebase;
  private double tagID = 1;

  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private String limelight;

  public AlignToReefTagRelative(int side, SwerveSubsystem drivebase, String limelight) {
    xController = new PIDController(Constants.VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0.04);  // Vertical movement
    yController = new PIDController(Constants.VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.04);  // Horitontal movement
    rotController = new PIDController(Constants.VisionConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.side = side;
    this.drivebase = drivebase;
    this.limelight = limelight;

    xSpeed = 0;
    ySpeed = 0;
    rotSpeed = 0;

    addRequirements(drivebase);
    
  }


  @Override
  public void initialize() {
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

    new SetAprilTagFilter(drivebase, limelight);

    tagID = LimelightHelpers.getFiducialID(limelight);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelight) && LimelightHelpers.getFiducialID(limelight) == tagID) {
      System.out.println("sees smth");

      double[] postions = LimelightHelpers.getBotPose_TargetSpace(limelight);
      SmartDashboard.putNumber("x", postions[2]);

      xSpeed = xController.calculate(postions[2]) / 2;
      SmartDashboard.putNumber("xspeed", xSpeed);
      ySpeed = -yController.calculate(postions[0]) / 2;
      rotSpeed = -rotController.calculate(postions[4]);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, false, true);

    } else {
      drivebase.drive(new Translation2d(0.0, 0.0), 0, false);
      xSpeed = 0.0;
      ySpeed = 0.0;
      
      System.out.println("no sees smth");
      System.out.println("SPEEDS ARE: " + xSpeed + "," + ySpeed);
    }
  }

    @Override
    public boolean isFinished() {
      return Math.abs(xSpeed) <= 0.05 && Math.abs(ySpeed) <= 0.05;
    }

  }

