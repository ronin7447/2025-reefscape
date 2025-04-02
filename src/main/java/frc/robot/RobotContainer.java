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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ShootCoral;
import frc.robot.commands.swervedrive.auto.AutoAlignCommand;
import frc.robot.commands.swervedrive.drivebase.AlignToCoralStationTagRelative;
import frc.robot.commands.swervedrive.drivebase.AlignToReefTagRelative;
import frc.robot.commands.swervedrive.drivebase.MoveALittle;
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
  final GenericHID XboxInfo = new GenericHID(0);

  // Secondary Operator Controller
  final Joystick driverPXN = new Joystick(1);


  
  // //DigitalInput L1_DIOInput = new DigitalInput(7);
  // DigitalInput L2_DIOInput = new DigitalInput(8);
  // DigitalInput L3_DIOInput = new DigitalInput(9);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

      // PIDController aling = new PIDController(.1, 0, 0);
      // ProfiledPIDController align = new ProfiledPIDController(.1, 0, 10, new Constraints(0.2, 2));
      // PIDController close = new PIDController(.1, 0.0, 2);
      // ProfiledPIDController translationalign = new ProfiledPIDController(0.6, 0, 0, new Constraints(0.2, 2));

      ProfiledPIDController rotationalign = new ProfiledPIDController(0.1, 0, 2, new Constraints(0.2, 0.2));

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
  private final Vision visionSubsystem = new Vision();

  // private final AutoAlignCommand autoAlignCommand = new AutoAlignCommand(drivebase, visionSubsystem, close);
  private final AlignToReefTagRelative alignToReefTagRelativeLeft = new AlignToReefTagRelative(0, drivebase, Constants.LimelightConstants.FRONTLL);
  private final AlignToReefTagRelative alignToReefTagRelativeCenter = new AlignToReefTagRelative(1, drivebase, Constants.LimelightConstants.FRONTLL);
  private final AlignToReefTagRelative alignToReefTagRelativeRight = new AlignToReefTagRelative(2, drivebase, Constants.LimelightConstants.FRONTLL);

  private final AlignToCoralStationTagRelative alignToCoral = new AlignToCoralStationTagRelative(1, drivebase, Constants.LimelightConstants.BACKLL);

  private final MoveALittle goLeftALittle = new MoveALittle(drivebase, 0);
  private final MoveALittle goRightALittle = new MoveALittle(drivebase, 1);

  


  // the current mode for debug message, 0 is field centric, 1 is robot centric
  private int currentMode = 0; 


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  //       () -> -driverXbox.getLeftY(),
  //       () -> -driverXbox.getLeftX())
  //       // .withControllerRotationAxis(() -> -1 * driverXbox.getRightX()) // rotation is inverted so we inverted the input :)
  //       .withControllerRotationAxis(() -> {
  //         if (driverXbox.getLeftTriggerAxis() > 0.5) {
  //           if (visionSubsystem.getTX() < 1.25 && visionSubsystem.getTX() > -1.25) {
  //             RobotLogger.log("Auto align completes");
  //             return 0.0;
  //           } else {
  //             RobotLogger.log("Auto align in progress, moving speed is: " + close.calculate(visionSubsystem.getTX(), 0));
  //             return close.calculate(visionSubsystem.getTX(), 0.0);
  //           }
  //         } else {
  //           return -1 * driverXbox.getRightX();
  //         }
  //       })
  //       .deadband(OperatorConstants.DEADBAND)
  //       .scaleTranslation(0.8)
  //       .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX())
        .withControllerRotationAxis(() -> -1 * driverXbox.getRightX()) // rotation is inverted so we inverted the input :)
        // .withControllerRotationAxis()
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(false);


  // if (decrease_held) { // do later kaden decrease speed
  //   driveAngularVelocity = Swer1veInputStream.of(drivebase.getSwerveDrive(),
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

    rotationalign.reset(0.0, 0.0);
    
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    NamedCommands.registerCommand("ShootCoral", shooterSubsystem.ShootCoral());

    NamedCommands.registerCommand("RemoveAlgae", shooterSubsystem.RemoveAlgae());

    NamedCommands.registerCommand("MoveToL1",
      (Commands.run(() -> {
        elevatorSubsystem.goToL1();
      }).until(() -> 
        elevatorSubsystem.getElevatorSpeed() == 0
      )));

    NamedCommands.registerCommand("MoveToL2",
      (Commands.run(() -> {
        elevatorSubsystem.goToL2();
      }).until(() -> 
        elevatorSubsystem.getElevatorSpeed() == 0
      )));

    NamedCommands.registerCommand("MoveToL3",
      (Commands.run(() -> {
        elevatorSubsystem.goToL3();
      }).until(() -> 
        elevatorSubsystem.getElevatorSpeed() == 0
      )));

    
    NamedCommands.registerCommand("AlgaeOut",
      (Commands.run(() -> {
       algaeSubsystem.algaeOut();
       System.out.println(algaeSubsystem.getAlgaeMotor() < -45);
      }).until(() -> 
        algaeSubsystem.getAlgaeMotor() == 0
      )));

    NamedCommands.registerCommand("AlgaeIn",
      (Commands.run(() -> {
       algaeSubsystem.algaeIn();
       System.out.println(algaeSubsystem.getAlgaeMotor() > 0);
      }).until(() -> 
        algaeSubsystem.getAlgaeMotor() == 0
      )));
      
    NamedCommands.registerCommand("GoLeftALittle", goLeftALittle.until(() -> goLeftALittle.isFinished()));
    
    NamedCommands.registerCommand("GoRightALittle", goRightALittle.until(() -> goRightALittle.isFinished()));

    NamedCommands.registerCommand("LimeLightAlignLeft", alignToReefTagRelativeLeft.until(() -> alignToReefTagRelativeLeft.isFinished()));

    NamedCommands.registerCommand("LimeLightAlignCenter", alignToReefTagRelativeCenter.until(() -> alignToReefTagRelativeCenter.isFinished()));

    NamedCommands.registerCommand("LimeLightAlignRight", alignToReefTagRelativeRight.until(() -> alignToReefTagRelativeRight.isFinished()));

    NamedCommands.registerCommand("LimeLightAlignCoral", alignToCoral.until(() -> alignToCoral.isFinished()));


   
  }

  public double robotGetElevatorSpeed() {
    return elevatorSubsystem.getElevatorSpeed();
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
      // driverXbox.leftTrigger().whileTrue((Commands.runOnce(() -> {
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

      // Limelight code

      // final Pose2d[] poseHolder = new Pose2d[1];
      // driverXbox.b().whileTrue((new FunctionalCommand(()-> {}, ()-> {
      //   drivebase.drive(new ChassisSpeeds(0, 0, close.calculate(visionSubsystem.getTX(), 0)));
      // }, (bool)-> {
      //   poseHolder[0] = drivebase.getPose();
      //   System.out.println(poseHolder[0]);
      // }, ()-> visionSubsystem.getTX() < 3 && visionSubsystem.getTX() > -3, drivebase)));

      // driverXbox.leftTrigger().whileTrue((new FunctionalCommand(()-> {}, ()-> { // move command
      //   if (visionSubsystem.getTA() < 12.0) {
      //     drivebase.drive(new ChassisSpeeds(translationalign.calculate(visionSubsystem.getTA(), 15.0), 0, 0));
      //   }
      // }, (bool)-> {
      //   System.out.println("TA");
      //   System.out.println(visionSubsystem.getTA());
      //   System.out.println("finished. moving back to pose");
      //   System.out.println(poseHolder[0]);
      //   System.out.println(drivebase.getPose());
      //   // drivebase.driveToPose(poseHolder[0]);
      // }, () -> visionSubsystem.getTA() > 12.0, drivebase)));

      // Alignment command new
      // driverXbox.leftTrigger().onTrue( // Supposed to aim while you can move with joystick(s)
      //   new AutoAlignCommand(drivebase, visionSubsystem, 0.0, 0.0)
      //     .repeatedly()
      //   );






      // Debug elevator code




      driverXbox.button(9).onTrue((Commands.runOnce(() -> {
        algaeSubsystem.resetAlgaeMotor();
      })));

      driverXbox.povRight().onTrue((Commands.runOnce(() -> {
        // System.out.println("L1: " + elevatorSubsystem.positions[0]);
        // System.out.println("L2: " + elevatorSubsystem.positions[1]);
        // System.out.println("L3: " + elevatorSubsystem.positions[2]);
        // System.out.println(elevatorSubsystem.getElevatorPosition());
        // System.out.println("Current level: " + elevatorSubsystem.getLevel());
        System.out.println("Rotations: " + elevatorSubsystem.getElevatorHeight());
        System.out.println("Algae Position:" +  algaeSubsystem.getAlgaeDebug());
      })));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_7)
        .onTrue((Commands.run(() -> {
          elevatorSubsystem.goToL0();
        }).until(() -> 
          elevatorSubsystem.getElevatorSpeed() == 0
        )));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_8)
        .onTrue((Commands.run(() -> {
          elevatorSubsystem.goToL1();
        }).until(() -> 
          elevatorSubsystem.getElevatorSpeed() == 0
        )));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_9)
        .onTrue((Commands.run(() -> {
          elevatorSubsystem.goToL2();
        }).until(() -> 
          elevatorSubsystem.getElevatorSpeed() == 0
        )));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_10)
        .onTrue((Commands.run(() -> {
          elevatorSubsystem.goToL3();
        }).until(() -> 
          elevatorSubsystem.getElevatorSpeed() == 0
        )));

      // driverXbox.leftBumper()
      // .onTrue((Commands.run(() -> {
      //   elevatorSubsystem.goToL1();
      // }).until(() -> 
      //   elevatorSubsystem.getElevatorSpeed() == 0
      // )));

      driverXbox.leftBumper()
        .whileTrue(new AlignToReefTagRelative(0, drivebase, Constants.LimelightConstants.FRONTLL));
      driverXbox.rightBumper()
        .whileTrue(new AlignToReefTagRelative(2, drivebase, Constants.LimelightConstants.FRONTLL));

      driverXbox.y()
        .whileTrue(new AlignToReefTagRelative(1, drivebase, Constants.LimelightConstants.BACKLL));
      
      driverXbox.x()
        .onTrue((Commands.runOnce(() -> {
          elevatorSubsystem.setEncoderPos(1.0 - Math.abs(elevatorSubsystem.getElevatorHeight()));
        })));


      // Shoot Coral
      driverXbox.rightTrigger()
        .whileTrue(new ShootCoral(1, shooterSubsystem));
      

      // THIS IS FOR ELEVATOR POSITION RESET, [!IMPORTANT!]
      // WHEN PRESSING BOTH BACK AND START BTNSs
      // driverXbox.back()
      // .and(driverXbox.start()).onTrue((Commands.runOnce(() -> {
      //   RobotLogger.warning("Elevator encoder position has been reset!");
      //   elevatorSubsystem.resetPosition();
      // })));

      
      // driverXbox.povLeft().onTrue((Commands.runOnce(() -> {
      //   elevatorSubsystem.goToTrueZero();
      // })));
      
      // driverXbox.y().onTrue((Commands.runOnce(() -> {
      //   algaeSubsystem.runAlgaeMotor(Constants.AlgaeConstants.ALGAE_SPEED);
      // })));

      // driverXbox.y().onFalse((Commands.runOnce(() -> {
      //   algaeSubsystem.stopAlgaeMotor();
      // })));

      // driverXbox.a().onTrue((Commands.runOnce(() -> {
      //   algaeSubsystem.runAlgaeMotor(Constants.AlgaeConstants.ALGAE_REVERSE_SPEED);
      // })));

      // driverXbox.a().onFalse((Commands.runOnce(() -> {
      //   algaeSubsystem.stopAlgaeMotor();
      // })));

      // VERY IMPORTANT FOR BACKUP,
      // UNRESTRCITED MOVING THE ELEVATOR,
      // SHOULD NOT USE IT UNLESS HAVE TO.

  
      driverXbox.povUp().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_SLOW_SPEED, true);
      })));

      driverXbox.povUp().onFalse((Commands.runOnce(() -> {
        elevatorSubsystem.stopElevatorMotor();
      })));

      driverXbox.povDown().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.runElevatorMotor(-1 * Constants.ElevatorConstants.ELEVATOR_SLOW_SPEED, true);
      })));

      driverXbox.povDown().onFalse((Commands.runOnce(() -> {
        elevatorSubsystem.stopElevatorMotor();
      })));



      // KADENS DEBUG STUFF
      // driverXbox.povUp().whileTrue((Commands.runOnce(() -> {
      //   System.out.println("pov L1: " + elevatorSubsystem.positions[0]);
      //   System.out.println("pov L2: " + elevatorSubsystem.positions[1]);
      //   System.out.println("pov L3: " + elevatorSubsystem.positions[2]);
      // })));

      // driverXbox.povDown().whileTrue((Commands.runOnce(() -> {
      //   System.out.println("pov L1: " + elevatorSubsystem.positions[0]);
      //   System.out.println("pov L2: " + elevatorSubsystem.positions[1]);
      //   System.out.println("pov L3: " + elevatorSubsystem.positions[2]);
      // })));

      driverXbox.start().onTrue((Commands.runOnce(() -> {
        drivebase.zeroGyro(); 
        System.out.println("GYRO HAS BEEN RESET!");
      })));

      driverXbox.back().onTrue((Commands.runOnce(() -> {
        elevatorSubsystem.setEncoderPos(0.86572265625);
        System.out.println("ELEVATOR HAS BEEN SET TO L1!");
      })));

      
      // Main PXN driver controls

      // new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_1) // Purple button 1
      //   .onTrue((Commands.runOnce(() -> {
      //     shooterSubsystem.runShooterMotor(Constants.ShooterConstants.SHOOTER_SPEED_HIGH);
      // })));

      // new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_1) // Purple button 1
      //   .onFalse((Commands.runOnce(() -> {
      //     shooterSubsystem.stopShooterMotor();
      // })));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_1)
        .whileTrue(new ShootCoral(-1, shooterSubsystem));

      // driverXbox.rightBumper()
      //   .whileTrue(new ShootCoral(-1, shooterSubsystem));



      
      // new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_1)
      //   .onFalse(Commands.runOnce(() -> {
      //     // Only go to L1 if button 10 is not pressed
      //     if (driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_10)) {
      //       // if the button 10 is pressed, go to l3
      //       RobotLogger.log("Button 10 is pressed, going to L3");
      //       elevatorSubsystem.goToL3();
      //     } else if (driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_9)) {
      //       // if the button 9 is pressed, go to l2
      //       RobotLogger.log("Button 9 is pressed, going to L2");
      //       elevatorSubsystem.goToL2();
      //     } else {
      //       RobotLogger.log("No button is pressed, going to L1");
      //       elevatorSubsystem.goToL1();
      //     }
      // }));


      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_5) // Leftest button 5
        .whileTrue(new ClimbCommand(1, climbSubsystem));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_3) // Pink button 3
        .whileTrue(new ClimbCommand(-1, climbSubsystem));


      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_4) // Leftest button 5
        .onTrue((Commands.runOnce(() -> {
          algaeSubsystem.runAlgaeMotor(Constants.AlgaeConstants.ALGAE_REVERSE_SPEED);
        })));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_4) // Leftest button 5
        .onFalse((Commands.runOnce(() -> {
          algaeSubsystem.stopAlgaeMotor();
        })));



      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_6) // Pink button 3
      .onTrue((Commands.runOnce(() -> {
        algaeSubsystem.runAlgaeMotor(Constants.AlgaeConstants.ALGAE_SPEED);
      })));

      new JoystickButton(driverPXN, Constants.OperatorConstants.BUTTON_6) // Pink button 3
      .onFalse((Commands.runOnce(() -> {
        algaeSubsystem.stopAlgaeMotor();
      })));
    
    
      //SLOW MOVEMENT

      new POVButton(driverPXN, 0)
        .whileTrue(new SlowDrive(drivebase, 0, true));

      new POVButton(driverPXN, 90)
        .whileTrue(new SlowDrive(drivebase, 90, false));

      new POVButton(driverPXN, 180)
        .whileTrue(new SlowDrive(drivebase, 180, true));
        
      new POVButton(driverPXN, 270)
        .whileTrue(new SlowDrive(drivebase, 270, false));


      // CLIMB JOYSTICK MODE (everything +-90 degrees)

      new Trigger(() -> driverPXN.getRawAxis(Constants.OperatorConstants.AXIS_Y) == -1) // -1 is up for some reason..?
        .whileTrue(new SlowDrive(drivebase, 270, true));

      new Trigger(() -> driverPXN.getRawAxis(Constants.OperatorConstants.AXIS_X) == 1)
        .whileTrue(new SlowDrive(drivebase, 0, true));

      new Trigger(() -> driverPXN.getRawAxis(Constants.OperatorConstants.AXIS_Y) == 1) // 1 is up for some reason..?
        .whileTrue(new SlowDrive(drivebase, 90, true));
    
      new Trigger(() -> driverPXN.getRawAxis(Constants.OperatorConstants.AXIS_X) == -1)
        .whileTrue(new SlowDrive(drivebase, 180, true));



      new Trigger(() -> !elevatorSubsystem.getLimitSwitch())
        .whileTrue((Commands.runOnce(() -> {
          elevatorSubsystem.setEncoderPos(0.0);
        })));

      new Trigger(() -> elevatorSubsystem.getIsElevatorMoving())
        .onTrue((Commands.runOnce(() -> {
          Constants.isReducedSpeed = true;
        })))
        .onFalse((Commands.runOnce(() -> {
          Constants.isReducedSpeed = false;
        })));

      // Drag code
      // new Trigger(() -> 
      //   elevatorSubsystem.getElevatorPosition() - Constants.ElevatorConstants.distanceToEncoder[0] < 0 &&
      //   elevatorSubsystem.currentLevel == 1 &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_8) &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_9) &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_10) &&
      //   XboxInfo.getPOV() == -1)
      //     .whileTrue((Commands.runOnce(() -> { 
      //       elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 4);
      //     })));

      // new Trigger(() -> 
      //   elevatorSubsystem.getElevatorPosition() - Constants.ElevatorConstants.distanceToEncoder[1] < 0 &&
      //   elevatorSubsystem.currentLevel == 2 &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_8) &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_9) &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_10) &&
      //   XboxInfo.getPOV() == -1)
      //     .whileTrue((Commands.runOnce(() -> {
      //       elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 4);
      //     })));

      // new Trigger(() -> 
      //   elevatorSubsystem.getElevatorPosition() - Constants.ElevatorConstants.distanceToEncoder[2] < 0 &&
      //   elevatorSubsystem.currentLevel == 3 &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_8) &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_9) &&
      //   !driverPXN.getRawButton(Constants.OperatorConstants.BUTTON_10) &&
      //   XboxInfo.getPOV() == -1)
      //     .whileTrue((Commands.runOnce(() -> { 
      //       elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 4);
      //     })));

      // new Trigger(() -> (elevatorSubsystem.getElevatorPosition() - Constants.ElevatorConstants.distanceToEncoder[2] < 0 &&
      // elevatorSubsystem.currentLevel == 3 &&
      // elevatorSubsystem.getElevatorSpeed() == 0))
      //     .whileTrue((Commands.runOnce(() -> { 
      //       elevatorSubsystem.runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 4);
      //     })));

      // new Trigger(() -> 
      //   !elevatorSubsystem.L1_DIOInput.get() ||
      //   !elevatorSubsystem.L2_DIOInput.get() ||
      //   !elevatorSubsystem.L3_DIOInput.get()
      //   )
      //     .whileTrue((Commands.run(() -> { 
      //       // elevatorSubsystem.setLevel();
      //       // System.out.println("HOLY MOLY THE CODE WORLKS");
      //       // System.out.println("new L1: " + elevatorSubsystem.positions[0]);
      //       // System.out.println("new L2: " + elevatorSubsystem.positions[1]);
      //       // System.out.println("new L3: " + elevatorSubsystem.positions[2]);
      //     })));


    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run autonomous command that made by PathPlanner
    return drivebase.getAutonomousCommand("AVRL2AlgaeRemover");

  }

  // public Command getAutoAlignCommand() {
  //   return AutoAlignCommand(drivebase, visionSubsystem, close);
  // }


  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
