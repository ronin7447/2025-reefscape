package frc.robot.commands.swervedrive.drivebase;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotLogger;

public class SetAprilTagFilter extends Command {
    // private SwerveSubsystem drivebase;
    private double tagID = -1;
    private String limelight;
    private NetworkTable limelightTable;


    public SetAprilTagFilter(String limelight, double tagID) {
        this.limelight = limelight;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelight);
        this.tagID = tagID;
    }

    @Override
    public void initialize() {
        // This method is called once when the command is scheduled
        double[] tagIds = limelightTable.getEntry("tid").getDoubleArray(new double[0]);
        double largestId = -1;

        if (tagID != -1) {
            // the target id is already set
            largestId = tagID;
            SmartDashboard.putNumber("Target AprilTag ID", largestId);
            RobotLogger.log("Target AprilTag ID set to: " + largestId);
            limelightTable.getEntry("pipeline").setNumber(0);
            limelightTable.getEntry("priorityid").setNumber(largestId);
        }

        if (tagIds.length > 0) {
            largestId = Arrays.stream(tagIds).max().orElse(-1);
            SmartDashboard.putNumber("Largest AprilTag ID Found", largestId);
            RobotLogger.log("Largest AprilTag ID found: " + largestId);

            if (largestId != -1) {
                // Assuming pipeline 8 is configured for AprilTag targeting
                limelightTable.getEntry("pipeline").setNumber(0);
                limelightTable.getEntry("priorityid").setNumber(largestId);
                SmartDashboard.putNumber("Limelight Target ID Set To", largestId);
                RobotLogger.log("Limelight filter set to target AprilTag ID: " + largestId);
            } else {
                RobotLogger.log("No valid AprilTag ID found to set the filter.");
                SmartDashboard.putString("Limelight Status", "No valid AprilTag ID found");
            }
        } else {
            RobotLogger.log("No AprilTags detected.");
            SmartDashboard.putString("Limelight Status", "No AprilTags detected");
        }
    }

    @Override
    public void execute() {
        // This method is called repeatedly while the command is scheduled
        // For this specific task, the filtering happens in initialize(),
        // so we might not need anything here unless we want to continuously update.
    }

    @Override
    public void end(boolean interrupted) {
        // This method is called once when the command ends or is interrupted
        if (interrupted) {
            RobotLogger.log("SetReefAprilTag command was interrupted.");
        }
    }

    @Override
    public boolean isFinished() {
        // This command should finish after setting the filter once
        return true;
    }
    
}
