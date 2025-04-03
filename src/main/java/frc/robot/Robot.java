// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;
  private Thread m_visionThread;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // private final LEDPattern m_ok = LEDPattern.solid(Color.kGreen);

  private final LEDPattern m_ok = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(41, 203, 255), new Color(0, 0, 17));
  private final LEDPattern m_elevatorMoving = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(221, 255, 0), new Color(255, 149, 0));
  // green for shooter moving
  private final LEDPattern m_shooterMoving = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(0, 255, 0), new Color(41, 94, 41));

  // private final LEDPattern m_vok = LEDPattern.

  private final LEDPattern m_notok = LEDPattern.solid(Color.kRed);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(180);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer (sets up button bindings and commands)
    m_robotContainer = new RobotContainer();


    // ----- FRONT NETWORK CAMERA -----
    // Check if the front camera stream is reachable before starting capture.
    try {
      URL cameraUrl = new URL("http://172.28.0.1:5801/stream.mjpg");
      HttpURLConnection connection = (HttpURLConnection) cameraUrl.openConnection();
      connection.setRequestMethod("HEAD");
      connection.setConnectTimeout(1000); // 1 second timeout
      connection.connect();

      if (connection.getResponseCode() == HttpURLConnection.HTTP_OK) {
        // Start automatic capture for the front camera using its URL.
        CameraServer.startAutomaticCapture("front", "http://172.28.0.1:5801/stream.mjpg");
      } else {
        System.out.println("Front camera not reachable: " + connection.getResponseCode());
      }
    } catch (IOException e) {
      System.out.println("Failed to connect to front camera: " + e.getMessage());
    }

    // ----- BACK LIMELIGHT CAMERA -----

    PortForwarder.add(5801, "limelight-back.local", 5801);
    PortForwarder.add(5800, "limelight-back.local", 5800);
    java.util.logging.Logger.getLogger("edu.wpi.first.cameraserver.CameraServer").setLevel(java.util.logging.Level.SEVERE);

    // ----- USB CAMERA (SHOOTER CAMERA) -----
    // Start the USB camera capture and processing on a separate thread.
  //   m_visionThread = new Thread(() -> {
  //     try {
  //       // Start capture from the USB camera.
  //       // UsbCamera usbCamera = CameraServer.startAutomaticCapture("Shooter USB Camera", 0);
  //       // usbCamera.setResolution(640, 480);

  //       // Get a CvSink to capture frames and a CvSource to output processed frames.
  //       // CvSink cvSink = CameraServer.getVideo("Shooter USB Camera");
  //       // CvSource outputStream = CameraServer.putVideo("Shooter Camera", 640, 480);

  //       // Reuse a single Mat to reduce repeated memory allocation.
  //       // Mat frame = new Mat();

  //       // while (!Thread.interrupted()) {
  //       //   // Grab a frame from the USB camera.
  //       //   if (cvSink.grabFrame(frame) == 0) {
  //       //     outputStream.notifyError(cvSink.getError());
  //       //     continue;
  //       //   }

  //       //   // Draw a rectangle on the image for visualization.
  //       //   Imgproc.rectangle(frame, new Point(370, 0), new Point(370, 480), new Scalar(0, 0, 255), 2);

  //       //   outputStream.putFrame(frame);

  //         // Small delay to reduce CPU load and allow native resources to breathe.
  //         // Timer.delay(0.01);
  //       }
  //     } catch (Exception e) {
  //       e.printStackTrace();
  //     }
  //   });
  //   m_visionThread.setDaemon(true);
  //   m_visionThread.start();

    // Create a timer to manage disabling motor brake after a delay.
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    // Run the command scheduler for command management.
    // Update the buffer with the rainbow animation

    if (DriverStation.isAutonomous()) {
      m_scrollingRainbow.applyTo(m_ledBuffer);
    } else if (m_robotContainer.robotGetElevatorSpeed() != 0) {
      m_elevatorMoving.applyTo(m_ledBuffer);
    } else if (m_robotContainer.robotGetShooterSpeed() != 0) {
      m_shooterMoving.applyTo(m_ledBuffer);
    } else {
      m_ok.applyTo(m_ledBuffer);
    }

    // Set the LEDs
    m_led.setData(m_ledBuffer);

    CommandScheduler.getInstance().run();

    // Update front NetworkTable value (e.g., tx) on the SmartDashboard.
    double tx_val = NetworkTableInstance.getDefault().getTable("front")
                      .getEntry("tx").getDouble(0.0);
    SmartDashboard.putNumber("Front TX", tx_val);
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
