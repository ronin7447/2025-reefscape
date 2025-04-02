package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MoveALittle extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private int direction;
    // private final Joystick controller;
    private Pose2d targetPose;

    public MoveALittle(SwerveSubsystem swerveSubsystem, int direction) {
        this.swerveSubsystem = swerveSubsystem;
        this.direction = direction;

        addRequirements(swerveSubsystem);
        
    }
    @Override
    public void execute()
    
    {
        Pose2d currentPose = swerveSubsystem.getPose();
        if (direction == 0) {
            targetPose = currentPose.transformBy(new Transform2d(new Translation2d(0, Constants.VisionConstants.MOVEALITTLE_LEFT), new Rotation2d(0)));
        } else {
            targetPose = currentPose.transformBy(new Transform2d(new Translation2d(0, Constants.VisionConstants.MOVEALITTLE_RIGHT), new Rotation2d(0)));
        }
        
        swerveSubsystem.driveToPose(targetPose);
    }

    @Override
    public boolean isFinished() {
        // This command runs until the button is released.
        Pose2d currentPose = swerveSubsystem.getPose();

        // Calculate the translational error between the current and target positions.
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        // Calculate the rotational error between the current and target headings.
        double angleError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());

        // Finish the command if both the distance and angle errors are within tolerance.
        return distanceError < Constants.VisionConstants.DISTANCE_TOLERANCE && angleError < Constants.VisionConstants.ANGLE_TOLERANCE;
    }

}

