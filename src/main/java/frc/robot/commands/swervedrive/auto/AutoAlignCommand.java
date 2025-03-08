package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

public class AutoAlignCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Vision vision;
    private final double  vX, vY;
    private final double headingHorizontal, headingVertical;
    public AutoAlignCommand(SwerveSubsystem swerveSubsystem, Vision vision, double vX, double vY, double headingHorizontal, double headingVertical) {
        this.vision = vision;
        this.swerveSubsystem = swerveSubsystem;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;
        addRequirements(this.swerveSubsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
        final var rotLimelight = vision.limelight_aim_proportional();
        final var forwardLimelight = vision.limelight_range_proportional();
        
        // final var targetSpeed = swerveSubsystem.getTargetSpeeds(forwardLimelight, vY, headingHorizontal, headingVertical);

        // swerveSubsystem.drive(targetSpeed, rot_limelight);
        Translation2d translation = new Translation2d(forwardLimelight, vY);

        swerveSubsystem.drive(translation, rotLimelight, false);

    }
}

