// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import java.util.logging.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import java.util.logging.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;


import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotLogger;

public class ElevatorSubsystem extends SubsystemBase {

    private boolean IsElevatorMoving;

    private final CANcoder AbsEncoder;

    private final SparkMax ElevatorMotor;
    private final SparkMaxConfig ElevatorMotorConfig;

    private final DigitalInput ElevatorLimitSwitch;

    private int MostRecentLevel;

    // Elevator Absolute Encoder



    public ElevatorSubsystem() {
        IsElevatorMoving = false;
        ElevatorLimitSwitch = new DigitalInput(6);
        AbsEncoder = new CANcoder(30);
        ElevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTORID, MotorType.kBrushless);
        ElevatorMotorConfig = new SparkMaxConfig();

        // AbsEncoder.setPosition(1 - Math.abs(AbsEncoder.getPosition().getValue().in(Rotations)));



        ElevatorMotor.configure(ElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // ElevatorEncoder.setPosition(0);
    }

    public void setEncoderPos(double pos) {
        AbsEncoder.setPosition(pos);
    }

    // public void setMostRecentLevel(int level) {
    //     MostRecentLevel = level;
    // }

    // public int getMostRecentLevel() {
    //     return MostRecentLevel;
    // }

    public void printElevatorPos() {
        System.out.println(AbsEncoder.getPosition().getValue().in(Rotations));

    }

    public boolean getLimitSwitch() {
        return ElevatorLimitSwitch.get();
    }


    public Command MoveElevatorToL1() {
        return runOnce(() -> goToL1());
    }

    public Command MoveElevatorToL2() {
        return runOnce(() -> goToL2());
    }

    public Command MoveElevatorToL3() {
        return runOnce(() -> goToL3());
    }

    public void setMotorLimit(int upperLimit, int lowerLimit) {

        ElevatorMotorConfig.softLimit
            .forwardSoftLimit(upperLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(lowerLimit)
            .reverseSoftLimitEnabled(true);
        
        ElevatorMotor.configure(ElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getElevatorSpeed() {

        return ElevatorMotor.get();
    }

    public double getElevatorHeight() {

        return AbsEncoder.getPosition().getValue().in(Rotations);
    }

    public boolean getIsElevatorMoving() {
        return IsElevatorMoving;
    }

    public void runElevatorMotor(double speed, boolean bypass) {

        if (!bypass) {
            if (speed != 0) {
                IsElevatorMoving = true;
            } else {
                IsElevatorMoving = false;
            }
        }  

        if (!ElevatorLimitSwitch.get()) {   
            if (speed < 0) {
                ElevatorMotor.set(0.0);
            } else {
                ElevatorMotor.set(speed);
            }
        } else {
            ElevatorMotor.set(speed);
        }
    }

    public void runElevatorMotor(double speed) {
        if (getElevatorHeight() <= 0 && speed < 0) {
            stopElevatorMotor();
        } else {
            runElevatorMotor(speed, false);
        }
    }

    public void stopElevatorMotor() {

        ElevatorMotor.stopMotor();
    }

    public double getPIDElevatorSpeed(double startingPos, double endingPos, double currentPos) {
        double b = Constants.ElevatorConstants.BASE_SPEED;
        double a = 1.15;
        double h = 1.0;
        double w = Math.abs(startingPos - endingPos) * 1.25;
        double t = (startingPos + endingPos) / 2;
        double x = getElevatorHeight();

        double speed = (h - b) * Math.pow(a, -1 * Math.pow(4 * (x - t) / w, 4)) + b;

        if (startingPos > endingPos) {
            speed *= -1;
        }

        return speed;

    }


    public void goToL0() {
        // if (MostRecentLevel == 0) {
            RobotLogger.log("Elevator is moving to L0 (0.0) with location " + getElevatorHeight() + "and speed " + getElevatorSpeed());
            
            if (getElevatorHeight() > 0.1 && ElevatorLimitSwitch.get()) {
                runElevatorMotor(getPIDElevatorSpeed(Constants.ElevatorConstants.L3_ABS, 0.0, getElevatorHeight()));
            } else {
                RobotLogger.log("Elevator made it to L0 (0.0)");

                stopElevatorMotor();
            }
        // } else {
        //     RobotLogger.error("Elevator attempted to move to L0 but most recently pressed command is " + MostRecentLevel);
        // }
    }

    public void goToL1() {
        // if (MostRecentLevel == 1) {
            RobotLogger.log("Elevator is moving to L1 (" + Constants.ElevatorConstants.L1_ABS + ") with location " + getElevatorHeight() + "and speed " + getElevatorSpeed());
            System.out.println("Elevator is moving to L1 (" + Constants.ElevatorConstants.L1_ABS + ") with location " + getElevatorHeight() + "and speed " + getElevatorSpeed());

            if (getElevatorHeight() - Constants.ElevatorConstants.L1_ABS > 0.01) {
                System.out.println("moving from up to down");
                runElevatorMotor(getPIDElevatorSpeed(Constants.ElevatorConstants.L3_ABS, Constants.ElevatorConstants.L1_ABS, getElevatorHeight()));
            } else if (getElevatorHeight() - Constants.ElevatorConstants.L1_ABS < -0.01) {
                System.out.println("moving from 0 to l1");
                runElevatorMotor(getPIDElevatorSpeed(0, Constants.ElevatorConstants.L1_ABS, getElevatorHeight()));
            } else {
                stopElevatorMotor();
            }
        // } else {
        //     RobotLogger.error("Elevator attempted to move to L1 but most recently pressed command is " + MostRecentLevel);
        // }
    }

    public void goToL2() {
        // if (MostRecentLevel == 2) {

            RobotLogger.log("Elevator is moving to L2 (" + Constants.ElevatorConstants.L2_ABS + ") with location " + getElevatorHeight() + "and speed " + getElevatorSpeed());

            System.out.println("Elevator is moving to L2 (" + Constants.ElevatorConstants.L2_ABS + ") with location " + getElevatorHeight() + "and speed " + getElevatorSpeed());

            if (getElevatorHeight() - Constants.ElevatorConstants.L2_ABS > 0.025) {
                runElevatorMotor(getPIDElevatorSpeed(Constants.ElevatorConstants.L3_ABS, Constants.ElevatorConstants.L2_ABS, getElevatorHeight()));
            } else if (getElevatorHeight() - Constants.ElevatorConstants.L2_ABS < -0.025) {
                if (getElevatorHeight() < Constants.ElevatorConstants.L1_ABS) {
                    runElevatorMotor(0.6);
                } else {
                    runElevatorMotor(getPIDElevatorSpeed(Constants.ElevatorConstants.L1_ABS, Constants.ElevatorConstants.L2_ABS, getElevatorHeight()));
                }
            } else {
                stopElevatorMotor();
            }
        // } else {
        //     RobotLogger.error("Elevator attempted to move to L2 but most recently pressed command is " + MostRecentLevel);
        // }
    }

    public void goToL3() {

        // if (MostRecentLevel == 3) {
            RobotLogger.log("Elevator is moving to L3 (" + Constants.ElevatorConstants.L3_ABS + ") with location " + getElevatorHeight() + "and speed " + getElevatorSpeed());

            System.out.println("Elevator is moving to L2 (" + Constants.ElevatorConstants.L2_ABS + ") with location " + getElevatorHeight() + "and speed " + getElevatorSpeed());

            if (getElevatorHeight() - Constants.ElevatorConstants.L3_ABS > 0.025) {
                runElevatorMotor(-1.0);
            } else if (getElevatorHeight() - Constants.ElevatorConstants.L3_ABS < -0.025) {
                if (getElevatorHeight() < Constants.ElevatorConstants.L1_ABS) {
                    runElevatorMotor(0.6);
                } else {
                    runElevatorMotor(getPIDElevatorSpeed(Constants.ElevatorConstants.L1_ABS, Constants.ElevatorConstants.L3_ABS, getElevatorHeight()));
                }
            } else {
                stopElevatorMotor();
            }
        // } else {
        //     RobotLogger.error("Elevator attempted to move to L3 but most recently pressed command is " + MostRecentLevel);
        // }
    }
}
