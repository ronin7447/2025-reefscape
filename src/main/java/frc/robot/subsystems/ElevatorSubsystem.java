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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax ElevatorMotor;
    private final SparkMaxConfig ElevatorMotorConfig;
    private RelativeEncoder ElevatorEncoder;

    public ElevatorSubsystem() {

        ElevatorMotor = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTORID, MotorType.kBrushless);
        ElevatorMotorConfig = new SparkMaxConfig();
        ElevatorEncoder = ElevatorMotor.getEncoder();

        ElevatorMotorConfig.idleMode(IdleMode.kBrake);   
        ElevatorMotorConfig.softLimit
        .forwardSoftLimit(Constants.ElevatorConstants.L3_HEIGHT)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.ElevatorConstants.L1_HEIGHT)
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

    public void resetPosition() {

        ElevatorEncoder.setPosition(0);

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

    public void goToL2() {
        if (getElevatorPosition() < Constants.ElevatorConstants.L2_HEIGHT) {
            while (getElevatorPosition() < Constants.ElevatorConstants.L2_HEIGHT - 3) {
              runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED);
            }
        } else {
            while (getElevatorPosition() > Constants.ElevatorConstants.L2_HEIGHT + 3) {
              runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
            }
        }        
        stopElevatorMotor();
    }

    public void goToL3() {
        if (getElevatorPosition() < Constants.ElevatorConstants.L3_HEIGHT) {
            while (getElevatorPosition() < Constants.ElevatorConstants.L3_HEIGHT) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_UP_SPEED);
            }
        } else {
            while (getElevatorPosition() > Constants.ElevatorConstants.L3_HEIGHT) {
                runElevatorMotor(Constants.ElevatorConstants.ELEVATOR_DOWN_SPEED);
            }
        }
        stopElevatorMotor();
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

}