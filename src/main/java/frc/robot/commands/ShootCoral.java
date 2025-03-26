package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCoral extends Command {
    
    private final int direction;
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    public ShootCoral(int direction) {

        this.direction = direction;

    }

    @Override
    public void execute() {

        if (direction > 0) {
            shooterSubsystem.runShooterMotor(Constants.ShooterConstants.SHOOTER_SPEED_HIGH); // Maybe change to dynamic based on elevation level
        }
        if (direction < 0) {
            shooterSubsystem.runShooterMotor(Constants.ShooterConstants.SHOOTER_REVERSE_SPEED); // Algae
        }

    }

    @Override
    public void end(boolean interrupted)
    {
        shooterSubsystem.stopShooterMotor();

    }

    @Override
    public boolean isFinished() {
        // This command runs until the button is released.
        return false;
    }
}
