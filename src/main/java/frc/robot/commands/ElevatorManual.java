package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManual extends Command {
    
    private final int direction;
    private final ElevatorSubsystem elevatorSubsystem;
    
        public ElevatorManual(int direction, ElevatorSubsystem elevatorSubsystem) {
    
            this.elevatorSubsystem = elevatorSubsystem;
            this.direction = direction;

            addRequirements(elevatorSubsystem);

    }

    @Override
    public void execute() {

        if (direction > 0) {
            elevatorSubsystem.setMotorLimit(-10000, 10000);
            elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 4);
        }
        if (direction < 0) {
            elevatorSubsystem.setMotorLimit(-10000, 10000);
            elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 4);
        }

    }

    @Override
    public void end(boolean interrupted)
    {
        elevatorSubsystem.stopElevatorMotor();
    }

    @Override
    public boolean isFinished() {
        // This command runs until the button is released.
        return false;
    }
}
