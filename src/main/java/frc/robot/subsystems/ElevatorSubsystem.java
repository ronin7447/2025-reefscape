// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final CANcoder AbsEncoder;

    private final SparkMax ElevatorMotor;
    private final SparkMaxConfig ElevatorMotorConfig;
    private RelativeEncoder ElevatorEncoder;

    public DigitalInput L1_DIOInput;
    public DigitalInput L2_DIOInput;
    public DigitalInput L3_DIOInput;


    public double[] positions = {0, 0, 0};



    private double lastPosition;

    public int currentLevel;


    public ElevatorSubsystem() {

        AbsEncoder = new CANcoder(0, "*");
        ElevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTORID, MotorType.kBrushless);
        ElevatorMotorConfig = new SparkMaxConfig();
        ElevatorEncoder = ElevatorMotor.getEncoder();

        L1_DIOInput = new DigitalInput(7); // False means it sees it
        L2_DIOInput = new DigitalInput(8);
        L3_DIOInput = new DigitalInput(9);

        if (!L1_DIOInput.get()) {
            currentLevel = 1;
            lastPosition = ElevatorEncoder.getPosition();
        } else if (!L2_DIOInput.get()) {
            currentLevel = 2;
            lastPosition = ElevatorEncoder.getPosition();
        } else if (!L3_DIOInput.get()) {
            currentLevel = 3;
            lastPosition = ElevatorEncoder.getPosition();
        } else {
            currentLevel = 0;
        }

        ElevatorMotorConfig.idleMode(IdleMode.kBrake);
        ElevatorMotorConfig.softLimit
                .forwardSoftLimit(1000)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(-1000)
                .reverseSoftLimitEnabled(true);

        ElevatorMotor.configure(ElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // ElevatorEncoder.setPosition(0);
    }

    public void getElevatorPos() {
        System.out.println(AbsEncoder.getPosition().getValue().in(Rotations));

    }

    public void recordLevel() {
        lastPosition = ElevatorEncoder.getPosition();
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

    public void runElevatorMotor(double speed) {

        ElevatorMotor.set(speed);

    }

    public void stopElevatorMotor() {

        ElevatorMotor.stopMotor();

    }

    public void showPosition() {

        System.out.println(ElevatorEncoder.getPosition());

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

    public double getElevatorPosition() {

        return ElevatorEncoder.getPosition();

    }

    public void resetPosition() {

        ElevatorEncoder.setPosition(0);

    }

    public double getPIDElevatorSpeed(double startingPos, double endingPos, double currentPos) {
        double b = Constants.ElevatorConstants.BASE_SPEED;
        double a = Math.E;
        double h = 0.3;
        double w = Math.abs(startingPos - endingPos) * 1.75;
        double t = (startingPos + endingPos) / 2;
        double x = getElevatorPosition();

        double speed = (h - b) * Math.pow(a, -1 * Math.pow(4 * (x - t) / w, 4)) + b;

        if (startingPos > endingPos) {
            speed *= -1;
        }

        return speed;

    }

    public void setLevel() {

        if (!L1_DIOInput.get()) {
            currentLevel = 1;
            recordLevel();
            positions[0] = getElevatorPosition();
            positions[1] = getElevatorPosition() + (Constants.ElevatorConstants.distances[1] - Constants.ElevatorConstants.distances[0]);
            positions[2] = getElevatorPosition() + (Constants.ElevatorConstants.distances[2] - Constants.ElevatorConstants.distances[0]);
        } else if (!L2_DIOInput.get()) {
            currentLevel = 2;
            recordLevel();
            positions[0] = getElevatorPosition() - (Constants.ElevatorConstants.distances[1] - Constants.ElevatorConstants.distances[0]);
            positions[1] = getElevatorPosition();
            positions[2] = getElevatorPosition() + (Constants.ElevatorConstants.distances[2] - Constants.ElevatorConstants.distances[1]);
        } else if (!L3_DIOInput.get()) {
            currentLevel = 3;
            recordLevel();
            positions[0] = getElevatorPosition() - (Constants.ElevatorConstants.distances[2] - Constants.ElevatorConstants.distances[0]);
            positions[1] = getElevatorPosition() - (Constants.ElevatorConstants.distances[2] - Constants.ElevatorConstants.distances[1]);
            positions[2] = getElevatorPosition();
        } else {
            currentLevel = 0;
        }
    }

    public int getLevel() {

        return currentLevel;

    }

    // Be careful... NEVER start using the elevator when it's above L3!
    // Only runs when level is unknown
    public void calibrateElevator() { // Move up slowly until it hits a target
        if (currentLevel == 0) {

            while (L1_DIOInput.get() && L2_DIOInput.get() && L3_DIOInput.get()) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
            }

            stopElevatorMotor();

            setLevel();
        }
    }

    public void goToL1() {
        calibrateElevator();

        if (ElevatorEncoder.getPosition() - (positions[0] + Constants.ElevatorConstants.distanceToEncoder[0]) > 1) {

            runElevatorMotor(getPIDElevatorSpeed(positions[getLevel() - 1], positions[0] + Constants.ElevatorConstants.distanceToEncoder[0], getElevatorPosition()));
        } else if (ElevatorEncoder.getPosition() - (positions[0] + Constants.ElevatorConstants.distanceToEncoder[0]) < -1) {
           
            if (ElevatorEncoder.getPosition() - positions[0] < 0) {
              
                runElevatorMotor(getPIDElevatorSpeed(positions[getLevel() - 1], positions[0] + Constants.ElevatorConstants.distanceToEncoder[0], getElevatorPosition()));
            } else {

                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
            }
        } else {

            stopElevatorMotor();
        }
    }

    public void goToL2() {
        calibrateElevator();

        if (ElevatorEncoder.getPosition() - (positions[1] + Constants.ElevatorConstants.distanceToEncoder[1]) > 1) {

            runElevatorMotor(getPIDElevatorSpeed(positions[getLevel() - 1], positions[1] + Constants.ElevatorConstants.distanceToEncoder[1], getElevatorPosition()));
        } else if (ElevatorEncoder.getPosition() - (positions[1] + Constants.ElevatorConstants.distanceToEncoder[1]) < -1) {

            if (ElevatorEncoder.getPosition() - positions[1] < 0) {

                runElevatorMotor(getPIDElevatorSpeed(positions[getLevel() - 1], positions[1] + Constants.ElevatorConstants.distanceToEncoder[1], getElevatorPosition()));
            } else {
            
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
            }
        } else {

            stopElevatorMotor();
        }
    }

    public void goToL3() {
        calibrateElevator();

        if (ElevatorEncoder.getPosition() - (positions[2] + Constants.ElevatorConstants.distanceToEncoder[2]) > 1) {

            runElevatorMotor(getPIDElevatorSpeed(positions[getLevel() - 1], positions[2] + Constants.ElevatorConstants.distanceToEncoder[2], getElevatorPosition()));
        } else if (ElevatorEncoder.getPosition() - (positions[1] + Constants.ElevatorConstants.distanceToEncoder[1]) < -1) {

            if (ElevatorEncoder.getPosition() - positions[2] < 0) {

                runElevatorMotor(getPIDElevatorSpeed(positions[getLevel() - 1], positions[2] + Constants.ElevatorConstants.distanceToEncoder[2], getElevatorPosition()));
            } else {
            
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
            }
        } else {

            stopElevatorMotor();
        }
    }

    
}
