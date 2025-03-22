package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SlowDrive extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Joystick controller;
    private double moveX, moveY;

    public SlowDrive(SwerveSubsystem swerveSubsystem, Joystick controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;

        addRequirements(this.swerveSubsystem);
    }

    @Override
    public void execute()
    {
        // if (controller.getPOV() == 0) {
        //     moveX = 0.2;
        //     moveY = 0.0;
        // }

        // if (controller.getPOV() == 90) {
        //     moveX = 0.0;
        //     moveY = -0.2;
        // }

        // if (controller.getPOV() == 180) {
        //     moveX = -0.2;
        //     moveY = 0.0;
        // }

        // if (controller.getPOV() == 270) {
        //     moveX = 0.0;
        //     moveY = 0.2;
        // }

        // Daniel says this works in trig so whatever
        moveX = 0.2 * Math.cos(Math.toRadians(-controller.getPOV()));
        moveY = 0.2 * Math.sin(Math.toRadians(-controller.getPOV()));

        swerveSubsystem.driveCommand(
            () -> moveX,
            () -> moveY,
            () -> 0.0
        );
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

