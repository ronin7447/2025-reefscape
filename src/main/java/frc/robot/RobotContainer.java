// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Team 7447 Ronin Robotics
// Field Centric / Robot Centric Code (ver. 2025)

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.SlowDrive;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */

public class RobotContainer {

  // Primary Operator Controller
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // Secondary Operator Controller
  final Joystick driverPXN = new Joystick(1);

  DigitalInput L1_DIOInput = new DigitalInput(7);
  DigitalInput L2_DIOInput = new DigitalInput(8);
  DigitalInput L3_DIOInput = new DigitalInput(9);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

      PIDController aling = new PIDController(.1, 0, 0);
      ProfiledPIDController align = new ProfiledPIDController(.1, 0, 10, new Constraints(0.2, 2));
      ProfiledPIDController close = new ProfiledPIDController(.05, 0, 10, new Constraints(0.2, 2));
      ProfiledPIDController translationalign = new ProfiledPIDController(0.3, 0, 0.01, new Constraints(0.2, 2));

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
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

        SwerveInputStream autoAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> 0,
        () -> 0
        )
        .withControllerRotationAxis(() -> -1 * aling.calculate(visionSubsystem.getTX(), 0)) // rotation is inverted so we inverted the input :)
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
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -1 * driverXbox.getRightX()) // rotation is inverted so we inverted the input :)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

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

    Command driveAutoAngularVelocity = drivebase.driveFieldOriented(autoAngularVelocity);

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

      // Main Xbox driver controls

      // // Toggles between field centric and robot centric
      // driverXbox.a().toggleOnTrue((Commands.runOnce(drivebase::zeroGyro)).repeatedly());
      // driverXbox.a().onTrue((Commands.runOnce(() -> {
      //   // debug message
      //    System.out.println();
      //    if (currentMode == 0) {
      //      currentMode = 1;
      //      System.out.println("the mode is now: ROBOT CENTRIC");
      //    } else {
      //      currentMode = 0;
      //      System.out.println("the mode is now: FIELD CENTRIC");
      //    }
      //    System.out.println();
      //  })));

      
      // driverXbox.a().onTrue((Commands.runOnce(() -> {
      //   if (decrease_held == false) {
      //     decrease_held = true;
      //   } else {
      //     decrease_held = false;
      //   }
      // })));

      // auto align command
      // driverXbox.start().whileTrue((Commands.runOnce(() -> {
      //   System.out.println("Auto align start");
      //   //  AutoAlignCommand autoAlignCommand = new AutoAlignCommand(drivebase, visionSubsystem, 0, 0, 0, 0);
      //   //  autoAlignCommand.execute();
      //   System.out.println(visionSubsystem.getTA()); // FORWARD/BACKWARD
      //   // System.out.println(visionSubsystem.getTranslation()[1]); // LEFT/RIGHT
      //   // drivebase.drive(new ChassisSpeeds(translationalign.calculate(visionSubsystem.getTranslation()[0], 0), translationalign.calculate(-visionSubsystem.getTranslation()[1], 0), close.calculate(visionSubsystem.getTX(), 0)));
      //   new FunctionalCommand(()-> {}, ()-> {}, null, null, null)
      //   drivebase.drive(new ChassisSpeeds(translationalign.calculate(visionSubsystem.getTA(), 0.5), 0, 0));
      //   // if (visionSubsystem.getTX() > 6 || visionSubsystem.getTX() < -6) {
      //   //   drivebase.drive(new ChassisSpeeds(0, 0, close.calculate(visionSubsystem.getTX(), 0)));
      //   // } else {
      //   //   // drivebase.drive(new ChassisSpeeds(0, 0, 0));
      //   //   System.out.println(translationalign.calculate(visionSubsystem.getTA(), 0.5));
      //   //   drivebase.drive(new ChassisSpeeds(translationalign.calculate(visionSubsystem.getTA(), 0.5), 0, close.calculate(visionSubsystem.getTX(), 0)));
      //   // }
      // })).repeatedly());
      final Pose2d[] poseHolder = new Pose2d[1];
      driverXbox.start().whileTrue((new FunctionalCommand(()-> {}, ()-> {
        drivebase.drive(new ChassisSpeeds(0, 0, close.calculate(visionSubsystem.getTX(), 0)));
      }, (bool)-> {
        poseHolder[0] = drivebase.getPose();
        System.out.println(poseHolder[0]);
      }, ()-> visionSubsystem.getTX() < 3 && visionSubsystem.getTX() > -3, drivebase)));

      driverXbox.y().whileTrue((new FunctionalCommand(()-> {}, ()-> {
        drivebase.drive(new ChassisSpeeds(translationalign.calculate(-visionSubsystem.getTA(), 20.0), 0, 0));
      }, (bool)-> {
        System.out.println("TA");
        System.out.println(visionSubsystem.getTA());
        System.out.println("finished. moving back to pose");
        System.out.println(poseHolder[0]);
        System.out.println(drivebase.getPose());
        // drivebase.driveToPose(poseHolder[0]);
      }, ()->visionSubsystem.getTA() > 15.0, drivebase)));

      // Elevator Go to L1
      driverXbox.leftBumper().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setInitPos();
        elevatorSubsystem.setMotorLimit(Constants.ElevatorConstants.L3_HEIGHT, Constants.ElevatorConstants.L1_HEIGHT);

        while (!L1_DIOInput.get()) {
          elevatorSubsystem.goToL1();
        }
        elevatorSubsystem.stopElevatorMotor();
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
      
      // THIS IS FOR ELEVATOR POSITION RESET, [!IMPORTANT!]
      // WHEN PRESSING BOTH BACK AND START BTNSs
      driverXbox.back()
      .and(driverXbox.start()).onTrue((Commands.runOnce(() -> {
        System.out.println("Reset to 0");
        elevatorSubsystem.resetPosition();
      })));

      // driverXbox.povLeft().onTrue((Commands.runOnce(() -> {
      //   elevatorSubsystem.goToTrueZero();
      // })));
      

     

      // VERY IMPORTANT FOR BACKUP,
      // UNRESTRCITED MOVING THE ELEVATOR,
      // SHOULD NOT USE IT UNLESS HAVE TO.
      driverXbox.povUp().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setMotorLimit(1000, 0);
        elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 4);
      })));

      driverXbox.povUp().onFalse((Commands.runOnce(() -> {
        elevatorSubsystem.stopElevatorMotor();
      })));

      driverXbox.povDown().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setMotorLimit(1000, -1000);
        elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 4);
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

      
      // Main PXN driver controls
      
      // TEST BUTTON remove later pls! -kaden 3/20/2025
      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON1) // THE PURPLE ONE
        .onTrue((Commands.runOnce(() -> {
          shooterSubsystem.runShooterMotor(Constants.ShooterConstants.SHOOTER_SPEED_HIGH);
        })));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON1)
        .onFalse((Commands.runOnce(() -> {
          shooterSubsystem.stopShooterMotor();
      })));

      // Algae button forward
      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON2) // THE RED ONE IS FORWARD I THINK -kaden 3/20/2025
        .onTrue((Commands.runOnce(() -> {
          algaeSubsystem.runAlgaeMotor(Constants.AlgaeConstants.ALGAE_SPEED);
        })));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON2)
        .onFalse((Commands.runOnce(() -> {
          algaeSubsystem.stopAlgaeMotor();
      })));

      // Algae button backward
      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON4) // THE BLUE ONE IS BACKWARD I THINK -kaden 3/20/2025
        .onTrue((Commands.runOnce(() -> {
          algaeSubsystem.runAlgaeMotor(Constants.AlgaeConstants.ALGAE_REVERSE_SPEED);
        })));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON4)
        .onFalse((Commands.runOnce(() -> {
          algaeSubsystem.stopAlgaeMotor();
      })));
      

      // Set offsets for Limelight alignment
      /*
       * left (270 degrees)
       * normal/reset (0 degrees)
       * right (90 degrees)
       * 
       * I HAVE NOT TESTED THIS PLS TEST -KADEN 3/20/25
       */
      //CASE 1 - ALIGN WITH REEF BEFORE GOING IN
      // new POVButton(driverPXN, Constants.OperatorConstants.JOYSTICK_LEFT)
      //   .onTrue((Commands.runOnce(() -> {
      //     LimelightHelpers.setFiducial3DOffset("limelight-front", -0.16, 0, 0); // 0.16m is about the length from the center of april tag to left/right reef sticks
      // })));

      // new POVButton(driverPXN, Constants.OperatorConstants.JOYSTICK_UP)
      //   .onTrue((Commands.runOnce(() -> {
      //     LimelightHelpers.setFiducial3DOffset("limelight-front", 0, 0, 0);
      // })));
      
      // new POVButton(driverPXN, Constants.OperatorConstants.JOYSTICK_RIGHT)
      //   .onTrue((Commands.runOnce(() -> {
      //     LimelightHelpers.setFiducial3DOffset("limelight-front", 0.16, 0, 0);
      // })));
    
      //CASE 2 - SLOW MOVEMENT

      // Trigger povTrigger = new Trigger(() -> driverPXN.getPOV() != -1);

      // povTrigger.whileTrue(new SlowDrive(drivebase, driverPXN));


      new POVButton(driverPXN, 90)
        .onTrue((Commands.runOnce(() -> {
          drivebase.driveCommand(
            () -> 0.0,
            () -> 3.0,
            () -> 0.0
            ).repeatedly();
          System.out.println("THIS CODE IS RUNNINGKADEN");
      })));

    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run autonomous command that made by PathPlanner
    return drivebase.getAutonomousCommand("TestMoveanDRotate");
  }


  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
