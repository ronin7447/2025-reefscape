package frc.robot.commands.swervedrive.drivebase;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetAprilTagFilter extends Command {
    private SwerveSubsystem drivebase;
    private double tagID = -1;
    private String limelight;
    private NetworkTable limelightTable;


    public SetAprilTagFilter(SwerveSubsystem drivebase, String limelight, double tagID) {
        this.drivebase = drivebase;
        this.limelight = limelight;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelight);
        this.tagID = tagID;

        addRequirements(drivebase);
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
            System.out.println("Target AprilTag ID set to: " + largestId);
            limelightTable.getEntry("pipeline").setNumber(0);
            limelightTable.getEntry("tidx").setNumber(largestId);
        }

        if (tagIds.length > 0) {
            largestId = Arrays.stream(tagIds).max().orElse(-1);
            SmartDashboard.putNumber("Largest AprilTag ID Found", largestId);
            System.out.println("Largest AprilTag ID found: " + largestId);

            if (largestId != -1) {
                // Assuming pipeline 8 is configured for AprilTag targeting
                limelightTable.getEntry("pipeline").setNumber(0);
                limelightTable.getEntry("tidx").setNumber(largestId);
                SmartDashboard.putNumber("Limelight Target ID Set To", largestId);
                System.out.println("Limelight filter set to target AprilTag ID: " + largestId);
            } else {
                System.out.println("No valid AprilTag ID found to set the filter.");
                SmartDashboard.putString("Limelight Status", "No valid AprilTag ID found");
            }
        } else {
            System.out.println("No AprilTags detected.");
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
            System.out.println("SetReefAprilTag command was interrupted.");
        }
    }

    @Override
    public boolean isFinished() {
        // This command should finish after setting the filter once
        return true;
    }
    
}
