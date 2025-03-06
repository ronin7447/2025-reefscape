// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax ClimbMotor;
    private final SparkMaxConfig ClimbmotorConfig;
    //private RelativeEncoder climbEncoder;

    public ClimbSubsystem() {

        ClimbMotor = new SparkMax(Constants.ClimbConstants.CLIMB_MOTORID, MotorType.kBrushless);
        ClimbmotorConfig = new SparkMaxConfig();
        //ShooterEncoder = ShooterMotor.getEncoder();

        ClimbmotorConfig.idleMode(IdleMode.kBrake);

        ClimbMotor.configure(ClimbmotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //ShooterEncoder.setPosition(0);

    }

    public void runClimbMotor(double speed) {

        ClimbMotor.set(speed);

    }

    public void stopClimbMotor() {

        ClimbMotor.stopMotor();

    }
}