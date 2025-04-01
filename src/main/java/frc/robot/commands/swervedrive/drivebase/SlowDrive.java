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
    private boolean faster;

    public SlowDrive(SwerveSubsystem swerveSubsystem, int angle, boolean faster) {
        this.swerveSubsystem = swerveSubsystem;
        // this.controller = controller;
        this.angle = angle;

        this.faster = faster;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute()
    {

        double angleRadians = Math.toRadians(angle);

        moveX = Constants.DECREASED_SPEED * Math.cos(angleRadians);
        moveY = -1 * Constants.DECREASED_SPEED * Math.sin(angleRadians);

        if (faster) {
            moveX *= 1.5;
            moveY *= 1.5;
        }

        swerveSubsystem.drive(new Translation2d(moveX, moveY), 0.0, true);
        
        
    }

    @Override
    public void end(boolean interrupted)
    {
        swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, true);

    }

    @Override
    public boolean isFinished() {
        // This command runs until the button is released.
        return false;
    }

}

