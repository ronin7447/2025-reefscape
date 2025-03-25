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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax ElevatorMotor;
    private final SparkMaxConfig ElevatorMotorConfig;
    private RelativeEncoder ElevatorEncoder;

    //private DigitalInput L1_DIOInput;
    private DigitalInput L2_DIOInput;
    private DigitalInput L3_DIOInput;

    private double initPos;

    private int currentLevel;

    // private boolean L1bool = false; // Probably starts at false kaden3/21/25

    public ElevatorSubsystem() {

        ElevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTORID, MotorType.kBrushless);
        ElevatorMotorConfig = new SparkMaxConfig();
        ElevatorEncoder = ElevatorMotor.getEncoder();

        //L1_DIOInput = new DigitalInput(7);
        L2_DIOInput = new DigitalInput(8);
        L3_DIOInput = new DigitalInput(9);

        if (currentLevel > 100) { /*!L1_DIOInput.get()*/ // dummy code Daniel
            currentLevel = 1;
        } else if (!L2_DIOInput.get()) {
            currentLevel = 2;
        } else if (!L3_DIOInput.get()) {
            currentLevel = 3;
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

    public int getElevatorLevelFromEncoder() {
        double position = this.getElevatorPosition();
        if (position < Constants.ElevatorConstants.L1_HEIGHT) {
            return 0;
        } else if (position < Constants.ElevatorConstants.L2_HEIGHT) {
            return 1;
        } else if (position < Constants.ElevatorConstants.L3_HEIGHT) {
            return 2;
        } else {
            return 3;
        }
    }

    public void resetPosition() {

        ElevatorEncoder.setPosition(0);

    }

    public void setLevel(int currentLevel) {

        this.currentLevel = currentLevel;

    }

    public int getLevel() {

        return currentLevel;

    }

    public void goToL1() {
        if (getElevatorPosition() > Constants.ElevatorConstants.L1_HEIGHT) {
            while (getElevatorPosition() > Constants.ElevatorConstants.L1_HEIGHT) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
            }
        } else {
            while (getElevatorPosition() < Constants.ElevatorConstants.L1_HEIGHT) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED);
            }
        }
        stopElevatorMotor();
    }
    // public void setInitPos() {
    //     initPos = getElevatorPosition();
    // }
    
    // public void goToL1() {
    //     runElevatorMotor(getElevatorSpeed(getElevatorPosition(), initPos, Constants.ElevatorConstants.L1_HEIGHT, Constants.ElevatorConstants.BASE_SPEED));
    // }

    public void goToL2() {
        // if (getElevatorPosition() < Constants.ElevatorConstants.L2_HEIGHT) {
        //     while (getElevatorPosition() < Constants.ElevatorConstants.L2_HEIGHT ) {
        //       runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED);
        //     }
        // } else {
        //     while (getElevatorPosition() > Constants.ElevatorConstants.L2_HEIGHT) {
        //       runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
        //     }
        // }        
        // stopElevatorMotor();
        System.out.println("elevator is currently at (before)"+currentLevel);
        currentLevel = getElevatorLevelFromEncoder();
        System.out.println("elevator is currently at (from encoder update)"+currentLevel);
        if (currentLevel == 0) {
            // make sure to add L1.get()
            while (L2_DIOInput.get() && L3_DIOInput.get()) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
            }
        } else if (currentLevel > 2) {
          while (L2_DIOInput.get()) {
            runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 2);
          }
        } else if (currentLevel < 2) {
          while (L2_DIOInput.get()) {
            runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
          }
          
        }
        stopElevatorMotor();
        currentLevel = 2;

    }

    public void goToL3() {
        // if (getElevatorPosition() < Constants.ElevatorConstants.L3_HEIGHT) {
        //     while (getElevatorPosition() < Constants.ElevatorConstants.L3_HEIGHT) {
        //         runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED);
        //     }
        // } else {
        //     while (getElevatorPosition() > Constants.ElevatorConstants.L3_HEIGHT) {
        //         runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
        //     }
        // }
        // stopElevatorMotor();
        // runElevatorMotor(getElevatorSpeed(getElevatorPosition(), initPos, Constants.ElevatorConstants.L3_HEIGHT, Constants.ElevatorConstants.BASE_SPEED));
        System.out.println("elevator is currently at (before)"+currentLevel);
        currentLevel = getElevatorLevelFromEncoder();
        System.out.println("elevator is currently at (from encoder update)"+currentLevel);
        if (currentLevel == 0) {
            // make sure to add L1.get()
            while (L2_DIOInput.get() && L3_DIOInput.get()) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
            }
        } else if (currentLevel > 3) {
          while (L3_DIOInput.get()) {
            runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 2);
          }
        } else if (currentLevel < 3) {
          while (L3_DIOInput.get()) {
            runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED / 2);
          }
        }
        stopElevatorMotor();
        currentLevel = 3;
    
    }
    // public void goToTrueZero() {
    //     // while (getElevatorPosition() > Constants.ElevatorConstants.TRUE_BOTTOM + 40) {
    //     //     runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
    //     //     System.out.println
    //     // }
    //     // while (getElevatorPosition() > Constants.ElevatorConstants.TRUE_BOTTOM + 20) {
    //     //     runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 4);
    //     // }       
    //     // while (getElevatorPosition() > Constants.ElevatorConstants.TRUE_BOTTOM + 10) {
    //     //     runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 8);
    //     // }    
    //     // while (getElevatorPosition() > Constants.ElevatorConstants.TRUE_BOTTOM + 3) {
    //     //     runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 12);
    //     // }    
    //     while (getElevatorPosition() > Constants.ElevatorConstants.TRUE_BOTTOM) {
    //         // System.out.println(this.getElevatorPosition());
    //         runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED / 8);
    //     }    
    //     stopElevatorMotor();
    // }

    public Command MoveElevatorToL1() {

        return runOnce(() -> goToL1());

    }

    public Command MoveElevatorToL2() {

        return runOnce(() -> goToL2());

    }

    public Command MoveElevatorToL3() {

        return runOnce(() -> goToL3());

    }

    public boolean getL2SensorStatus() {
        return L2_DIOInput.get();
    }

    public boolean getL3SensorStatus() {
        return L3_DIOInput.get();
    }

    // Get Elevator Speed
    // get speed by using a function to slow down when near the target
    public double getElevatorSpeed(double current, double start, double goal, double baseSpeed) {
        double speed = 0.0;

        if ((start < goal && current >= goal) || (start > goal && current <= goal)) {
            return 0.0;
        }

        double t = (start + goal) / 2;
        double w = 2.0;
        double h = 0.7;
        //speed = h * Math.pow(w, -((Math.pow(current-t, 2)) / 2)) + baseSpeed;

        speed = (h - baseSpeed) * Math.pow(w, Math.pow((-4 * (current - t)) / w, 4)) + baseSpeed;

        if (start > goal) {
            speed *= -1;
        }

        return speed;
    }

}