package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SlowDrive extends Command {
    private final SwerveSubsystem swerveSubsystem;
    // private final Joystick controller;
    private double moveX, moveY;
    private int angle;

    public SlowDrive(SwerveSubsystem swerveSubsystem, int angle) {
        this.swerveSubsystem = swerveSubsystem;
        // this.controller = controller;
        this.angle = angle;

        addRequirements(this.swerveSubsystem);
    }

    @Override
    public void execute()
    {
        if (angle == 0) { // Forward
            moveX = Constants.DECREASED_SPEED;
            moveY = 0.0;
        }
        if (angle == 90) { // Rightward
            moveX = 0.0;
            moveY = Constants.DECREASED_SPEED * -1;
        }
        if (angle == 180) { // Backward
            moveX = Constants.DECREASED_SPEED * -1;
            moveY = 0.0;
        }
        if (angle == 270) { // Leftward
            moveX = 0.0;
            moveY = Constants.DECREASED_SPEED;
        }

        swerveSubsystem.drive(new Translation2d(moveX, moveY), 0.0, true);
        
    }

    @Override
    public void end(boolean interrupted)
    {
        swerveSubsystem.driveCommand(
            () -> 0.0,
            () -> 0.0,
            () -> 0.0
        );

    }

    @Override
    public boolean isFinished() {
        // This command runs until the button is released.
        return false;
    }

}

