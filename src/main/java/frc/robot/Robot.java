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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;
  private Thread m_visionThread;

  public Robot() {
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

    // ----- USB CAMERA (SHOOTER CAMERA) -----
    // Start the USB camera capture and processing on a separate thread.
    m_visionThread = new Thread(() -> {
      try {
        // Start capture from the USB camera.
        UsbCamera usbCamera = CameraServer.startAutomaticCapture("Shooter USB Camera", 0);
        usbCamera.setResolution(640, 480);

        // Get a CvSink to capture frames and a CvSource to output processed frames.
        CvSink cvSink = CameraServer.getVideo("Shooter USB Camera");
        CvSource outputStream = CameraServer.putVideo("Shooter Camera", 640, 480);

        // Reuse a single Mat to reduce repeated memory allocation.
        Mat frame = new Mat();

        while (!Thread.interrupted()) {
          // Grab a frame from the USB camera.
          if (cvSink.grabFrame(frame) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }

          // Draw a rectangle on the image for visualization.
          Imgproc.rectangle(frame, new Point(370, 0), new Point(370, 480), new Scalar(0, 0, 255), 2);

          outputStream.putFrame(frame);

          // Small delay to reduce CPU load and allow native resources to breathe.
          Timer.delay(0.01);
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    // Create a timer to manage disabling motor brake after a delay.
    disabledTimer = new Timer();

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotPeriodic() {
    // Run the command scheduler for command management.
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
