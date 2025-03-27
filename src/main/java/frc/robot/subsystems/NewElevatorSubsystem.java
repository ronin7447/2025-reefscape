// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class NewElevatorSubsystem extends SubsystemBase {

    private final SparkMax ElevatorMotor;
    private final SparkMaxConfig ElevatorMotorConfig;
    private RelativeEncoder ElevatorEncoder;

    private DigitalInput L1_DIOInput;
    private DigitalInput L2_DIOInput;
    private DigitalInput L3_DIOInput;

    //private double initPos;

    private double[] positions = {0, 0, 0};

    private double lastPosition;

    private int currentLevel;

    // private boolean L1bool = false; // Probably starts at false kaden3/21/25

    public NewElevatorSubsystem() {

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

    public void recordLevel() {
        lastPosition = ElevatorEncoder.getPosition();
    }

    public void setMotorLimit(int upperLimit, int lowerLimit){
        ElevatorMotorConfig.softLimit
        .forwardSoftLimit(upperLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(lowerLimit)
        .reverseSoftLimitEnabled(true);

        ElevatorMotor.configure(ElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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

    public double getElevatorPosition() {

        return ElevatorEncoder.getPosition();
        
    }

    public void resetPosition() {

        ElevatorEncoder.setPosition(0);

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

        if (getLevel() == 3) {
            while(L2_DIOInput.get()) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
            }

            setLevel();
        }

        if (getLevel() == 2) {
            while (getElevatorPosition() > Constants.ElevatorConstants.distances[1] - Constants.ElevatorConstants.distances[0]) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
            }

            currentLevel = 2; // this is kinda weird

        }
    }

    public void goToL2() {
        calibrateElevator(); // Precaution

        if (getLevel() == 1) {
            while (L2_DIOInput.get()) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED);
            }

            setLevel();

            while (getElevatorPosition() < Constants.ElevatorConstants.distanceToEncoder[1] + Constants.ElevatorConstants.distances[1]) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2); // Half speed because it's nearly there
            };

            stopElevatorMotor();

        } else if (getLevel() == 3) {

            while (getElevatorPosition() > Constants.ElevatorConstants.distances[2] - Constants.ElevatorConstants.distances[1]) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
            }

            currentLevel = 2; // this is kinda weird
        }

    }

    public void goToL3() {
        calibrateElevator();

        if (getLevel() == 1) {
            while(L2_DIOInput.get()) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED);
            }

            setLevel();
        }
        if (getLevel() == 2) {
            while(L3_DIOInput.get()) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED);
            }

            setLevel();

            while (getElevatorPosition() < Constants.ElevatorConstants.distanceToEncoder[2] + Constants.ElevatorConstants.distances[2]) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
            }

            stopElevatorMotor();
        }

    }
}