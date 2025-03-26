package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

    private int direction;
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    public ClimbCommand(int direction) {

        this.direction = direction;

    }

    @Override
    public void execute()
    {
        
        if (direction > 0) {
            climbSubsystem.runClimbMotor(Constants.ClimbConstants.CLIMB_SPEED);
        }
        if (direction < 0) {
            climbSubsystem.runClimbMotor(Constants.ClimbConstants.CLIMB_REVERSE_SPEED);
        }
        
    }

    @Override
    public void end(boolean interrupted)
    {
        climbSubsystem.stopClimbMotor();

    }

    @Override
    public boolean isFinished() {
        // This command runs until the button is released.
        return false;
    }

}

