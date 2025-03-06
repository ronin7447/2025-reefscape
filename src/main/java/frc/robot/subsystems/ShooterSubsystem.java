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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax ShooterMotor;
    private final SparkMaxConfig ShooterMotorConfig;
    //private RelativeEncoder ShooterEncoder;

    public ShooterSubsystem() {

        ShooterMotor = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTORID, MotorType.kBrushless);
        ShooterMotorConfig = new SparkMaxConfig();
        //ShooterEncoder = ShooterMotor.getEncoder();

        ShooterMotorConfig.idleMode(IdleMode.kBrake);

        ShooterMotor.configure(ShooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //ShooterEncoder.setPosition(0);

    }

    public void runShooterMotor(double speed) {

        ShooterMotor.set(speed);

    }

    public void stopShooterMotor() {

        ShooterMotor.stopMotor();

    }

    public void shootCoral() {

        runShooterMotor(Constants.ShooterConstants.SHOOTER_SPEED_HIGH);
        Timer.delay(0.8);
        stopShooterMotor();

    }

    public Command ShootCoral() {

        return runOnce(() -> shootCoral());

    }

}