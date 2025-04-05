package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotLogger {
    private static final String LOG_FILE_PATH = "/home/lvuser/logs.txt";
    private static final DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss");

    private static String getTimestamp() {
        return LocalDateTime.now().format(formatter);
    }

    public static void log(String message) {
        String logMessage = "[" + getTimestamp() + "] [INFO] " + message;
        System.out.println(logMessage);
        // writeToDashboard(message);
        // writeToFile(logMessage);
    }

    public static void warning(String message) {
        String logMessage = "[" + getTimestamp() + "] [WARNING] " + message;
        System.out.println(logMessage);
        // writeToDashboard(message);
        // writeToFile(logMessage);
    }

    public static void error(String message) {
        String logMessage = "[" + getTimestamp() + "] [ERROR] " + message;
        System.err.println(logMessage);
        // writeToDashboard(message);
        // writeToFile(logMessage);
    }


    public static void updateDashboard(String title, String message) {
        String logMessage = "[" + getTimestamp() + "] [DASHBOARD] " + title + ": " + message;
        System.out.println(logMessage);
        SmartDashboard.putString(title, message);
        // writeToFile(logMessage);
    }


    // private static void writeToFile(String message) {
    //     try (FileWriter writer = new FileWriter(LOG_FILE_PATH, true)) {
    //         writer.write(message + "\n");
    //     } catch (IOException e) {
    //         System.err.println("[ERROR] Failed to write to log file: " + e.getMessage());
    //     }
    // }

    // private static void writeToDashboard(String message) {
    //     SmartDashboard.putString("Log", SmartDashboard.getString("Log", "") + message + "\n");
    // }
}

