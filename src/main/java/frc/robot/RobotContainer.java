// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Team 7447 Ronin Robotics
// Field Centric / Robot Centric Code (ver. 2025)

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import frc.robot.commands.swervedrive.auto.AutoAlignCommand;
import frc.robot.subsystems.swervedrive.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */

public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final Vision visionSubsystem = new Vision();
  


  // the current mode for debug message, 0 is field centric, 1 is robot centric
  private int currentMode = 0; 
  boolean decrease_held = false;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX())
        .withControllerRotationAxis(() -> -1 * driverXbox.getRightX()) // rotation is inverted so we inverted the input :)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(false);

  // if (decrease_held) { // do later kaden decrease speed
  //   driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //     () -> -driverXbox.getLeftY() / 2,
  //     () -> -driverXbox.getLeftX() / 2)
  //     .withControllerRotationAxis(() -> -1 * driverXbox.getRightX()) // rotation is inverted so we inverted the input :)
  //     .deadband(OperatorConstants.DEADBAND)
  //     .scaleTranslation(0.8)
  //     .allianceRelativeControl(false);
  // } else {
  //   driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //       () -> -driverXbox.getLeftY(),
  //       () -> -driverXbox.getLeftX())
  //       .withControllerRotationAxis(() -> -1 * driverXbox.getRightX()) // rotation is inverted so we inverted the input :)
  //       .deadband(OperatorConstants.DEADBAND)
  //       .scaleTranslation(0.8)
  //       .allianceRelativeControl(false);
  // }
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> -1 * driverXbox.getRightX(), // rotation is inverted so we inverted the input :)
          driverXbox::getRightY)
      .headingWhile(true);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY(),
      () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) * Math.PI)
          * (Math.PI * 2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) * Math.PI)
              *
              (Math.PI * 2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("ShootCoral", shooterSubsystem.ShootCoral());
    NamedCommands.registerCommand("MoveToL1", elevatorSubsystem.MoveElevatorToL1());
    NamedCommands.registerCommand("MoveToL2", elevatorSubsystem.MoveElevatorToL2());
    NamedCommands.registerCommand("MoveToL3", elevatorSubsystem.MoveElevatorToL3());
    
    // scheduleDebugCommand();
  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);
    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleSim);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }
    if (DriverStation.isTest()) {

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      // Main robot controls

      // Toggles between field centric and robot centric
      driverXbox.a().toggleOnTrue((Commands.runOnce(drivebase::zeroGyro)).repeatedly());
      driverXbox.a().onTrue((Commands.runOnce(() -> {
        // debug message
         System.out.println();
         if (currentMode == 0) {
           currentMode = 1;
           System.out.println("the mode is now: ROBOT CENTRIC");
         } else {
           currentMode = 0;
           System.out.println("the mode is now: FIELD CENTRIC");
         }
         System.out.println();
       })));

      
      driverXbox.a().onTrue((Commands.runOnce(() -> {
        if (decrease_held == false) {
          decrease_held = true;
        } else {
          decrease_held = false;
        }
      })));

      // auto align command
      driverXbox.start().whileTrue((Commands.runOnce(() -> {
        System.out.println("Auto align start");
         AutoAlignCommand autoAlignCommand = new AutoAlignCommand(drivebase, visionSubsystem, 0, 0, 0, 0);
         autoAlignCommand.execute();
      })));
      

      // Elevator Go to L1
      driverXbox.leftBumper().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setMotorLimit(Constants.ElevatorConstants.L3_HEIGHT, Constants.ElevatorConstants.L1_HEIGHT);
        elevatorSubsystem.goToL1();
      })));

      // Elevator Go to L3
      driverXbox.rightBumper().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setMotorLimit(Constants.ElevatorConstants.L3_HEIGHT, Constants.ElevatorConstants.L1_HEIGHT);
        elevatorSubsystem.goToL3();
      })));

      // Elevator Go to L2
      driverXbox.x().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setMotorLimit(Constants.ElevatorConstants.L3_HEIGHT, Constants.ElevatorConstants.L1_HEIGHT);
        elevatorSubsystem.goToL2();
      })));

      // Activate Shooter
      driverXbox.rightTrigger().onTrue((Commands.runOnce(() -> {
        if (elevatorSubsystem.getElevatorPosition() > 30) {
          shooterSubsystem.runShooterMotor(Constants.ShooterConstants.SHOOTER_SPEED_HIGH);
        } else {
          shooterSubsystem.runShooterMotor(Constants.ShooterConstants.SHOOTER_SPEED_LOW);
        }
      })));

      

      driverXbox.rightTrigger().onFalse((Commands.runOnce(() -> {
        shooterSubsystem.stopShooterMotor();
      })));


      // Reverse Shooter
      driverXbox.leftTrigger().onTrue((Commands.runOnce(() -> {
        shooterSubsystem.runShooterMotor(Constants.ShooterConstants.SHOOTER_REVERSE_SPEED);
      })));

      driverXbox.leftTrigger().onFalse((Commands.runOnce(() -> {
        shooterSubsystem.stopShooterMotor();
      })));
      
      // THIS IS FOR ELEVATOR POSITION RESET, [!IMPORTANT]
      // WHEN PRESSING BOTH BACK AND START BTNSs
      driverXbox.back()
      .and(driverXbox.start()).onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.resetPosition();
      })));

      // driverXbox.povLeft().onTrue((Commands.runOnce(() -> {
      //   elevatorSubsystem.goToTrueZero();
      // })));
      

      driverXbox.y().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.showPosition();
      })));

      // VERY IMPORTANT FOR BACKUP,
      // UNRESTRCITED MOVING THE ELEVATOR,
      // SHOULD NOT USE IT UNLESS HAVE TO.
      driverXbox.povUp().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setMotorLimit(1000, 0);
        elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
      })));

      driverXbox.povUp().onFalse((Commands.runOnce(() -> {
        elevatorSubsystem.stopElevatorMotor();
      })));

      driverXbox.povDown().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setMotorLimit(1000, -1000);
        elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 2);
      })));

      driverXbox.povDown().onFalse((Commands.runOnce(() -> {
        elevatorSubsystem.stopElevatorMotor();
      })));

      // Climb up
      driverXbox.povLeft().onTrue((Commands.runOnce(() -> {
        climbSubsystem.runClimbMotor(Constants.ClimbConstants.CLIMB_SPEED);
      })));

      driverXbox.povLeft().onFalse((Commands.runOnce(() -> {
        climbSubsystem.stopClimbMotor();
      })));

      // Climb down
      driverXbox.povRight().onTrue((Commands.runOnce(() -> {
        climbSubsystem.runClimbMotor(Constants.ClimbConstants.CLIMB_REVERSE_SPEED);
      })));

      driverXbox.povRight().onFalse((Commands.runOnce(() -> {
        climbSubsystem.stopClimbMotor();
      })));

      // Reset the Gyro to 0
      driverXbox.b().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.b().onTrue((Commands.runOnce(() -> {
        drivebase.zeroGyro();
        System.out.println("GYRO HAS BEEN RESET!");
      })));

      // driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run autonomous command that made by PathPlanner
    return drivebase.getAutonomousCommand("Test_Path_Complete");
  }


  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
