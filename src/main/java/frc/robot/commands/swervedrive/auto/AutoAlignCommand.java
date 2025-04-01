package frc.robot.commands.swervedrive.auto;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotLogger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class AutoAlignCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Vision vision;
    // private final double  vX, vY;
    // private final double headingHorizontal, headingVertical;
    private final ProfiledPIDController pid;
    
    public AutoAlignCommand(SwerveSubsystem swerveSubsystem, Vision vision, ProfiledPIDController pid) {
        this.vision = vision;
        this.swerveSubsystem = swerveSubsystem;
        // this.vX = vX;
        // this.vY = vY;
        // this.headingHorizontal = headingHorizontal;
        // this.headingVertical = headingVertical;
        this.pid = pid;
        addRequirements(this.swerveSubsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {

        double tx = vision.getTX();
        double rotationSpeed;
    
        // Check if we are within the alignment threshold
        if (Math.abs(tx) < 1.25) {
          RobotLogger.log("Auto align completes");
          rotationSpeed = 0.0;
        } else {
          rotationSpeed = pid.calculate(tx, 0);
          RobotLogger.log("Auto align in progress, moving speed is: " + rotationSpeed);
        }

        swerveSubsystem.drive(new Translation2d(0.0, 0.0), rotationSpeed, false);
    }

    @Override
    public boolean isFinished() {
        // End the command if the alignment error is within ±1.25
        return Math.abs(vision.getTX()) < 1.25;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion when the command ends
        swerveSubsystem.drive(new Translation2d(0.0, 0.0), 0.0, false);
    }

    

}

