// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotLogger;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax ClimbMotor;
    private final SparkMaxConfig ClimbmotorConfig;
    private final CANcoder ClimbEncoder;
    //private RelativeEncoder climbEncoder;

    public ClimbSubsystem() {

        ClimbMotor = new SparkMax(Constants.ClimbConstants.CLIMB_MOTORID, MotorType.kBrushless);
        ClimbEncoder = new CANcoder(Constants.ClimbConstants.CLIMB_ENCODERID);
        ClimbmotorConfig = new SparkMaxConfig();
        //ShooterEncoder = ShooterMotor.getEncoder();

        ClimbmotorConfig.idleMode(IdleMode.kBrake);

        ClimbMotor.configure(ClimbmotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //ShooterEncoder.setPosition(0);

    }

    public void runClimbMotor(double speed) {
        
        RobotLogger.log("Current position of climb encoder is: " + getClimbEncoder());

        ClimbMotor.set(speed);

    }

    public void stopClimbMotor() {

        ClimbMotor.stopMotor();

    }


    public void moveOutClimbMotor() {

        while (getClimbEncoder() < Constants.ClimbConstants.CLIMB_OUT_ENCODER_POSITION) {
            // for moving out the climb subsystem
            // it is a negative value of speed
            ClimbMotor.set(Constants.ClimbConstants.CLIMB_REVERSE_SPEED);
        }

        ClimbMotor.stopMotor();
        

    }

    public void moveInClimbMotor() {

        while (getClimbEncoder() > Constants.ClimbConstants.CLIMB_IN_ENCODER_POSITION) {
            // for moving in the climb subsystem
            // it is a positive value of speed
            ClimbMotor.set(Constants.ClimbConstants.CLIMB_SPEED);
        }
        
        ClimbMotor.stopMotor();
        
    }

    public double getClimbEncoder() {

        var position = ClimbEncoder.getPosition();

        // refresh the cache value of the position
        position.refresh();

        return position.getValueAsDouble();

    }
}