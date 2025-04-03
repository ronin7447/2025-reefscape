package frc.robot.commands.swervedrive.drivebase;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotLogger;

public class ResetAprilTagFilter extends Command {
    private SwerveSubsystem drivebase;
    private String limelight;
    private NetworkTable limelightTable;


    public ResetAprilTagFilter(SwerveSubsystem drivebase, String limelight) {
        this.drivebase = drivebase;
        this.limelight = limelight;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelight);
        addRequirements(drivebase);
    }
    @Override
    public void initialize() {
        limelightTable.getEntry("tidx").setNumber(-1);
        limelightTable.getEntry("pipeline").setNumber(0);
        RobotLogger.log("Limelight AprilTag filter reset (no specific ID target).");
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
            System.out.println("ResetReefAprilTag command was interrupted.");
        }
    }

    @Override
    public boolean isFinished() {
        // This command should finish after setting the filter once
        return true;
    }
    
}
