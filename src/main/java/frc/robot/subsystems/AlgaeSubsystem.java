// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command; //importd command

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkMax AlgaeMotor;
    private final RelativeEncoder AlgaeEncoder;
    private final SparkMaxConfig AlgaemotorConfig;

    public AlgaeSubsystem() {

        AlgaeMotor = new SparkMax(Constants.AlgaeConstants.ALGAE_MOTORID, MotorType.kBrushless);
        AlgaemotorConfig = new SparkMaxConfig();
        AlgaeEncoder = AlgaeMotor.getEncoder();

        AlgaemotorConfig.idleMode(IdleMode.kBrake);

        AlgaeMotor.configure(AlgaemotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public void runAlgaeMotor(double speed) {

        AlgaeMotor.set(speed);

    }

    public void resetAlgaeMotor() {
        AlgaeEncoder.setPosition(0);
    }

    public double getAlgaeDebug() {
        return AlgaeEncoder.getPosition();
    }

    public void stopAlgaeMotor() {

        AlgaeMotor.stopMotor();

    }

    public void algaeOut() {
        if (getAlgaeDebug() > -45) {
            runAlgaeMotor(Constants.AlgaeConstants.ALGAE_SPEED);
        } else {
            stopAlgaeMotor();
        }
    }

    public void algaeIn() {
        if (getAlgaeDebug() < 0) {
            runAlgaeMotor(Constants.AlgaeConstants.ALGAE_REVERSE_SPEED);
        } else {
            stopAlgaeMotor();
        }
    }

    public Command AlgaeOut() {
        return run(() -> algaeOut()).until(() -> getAlgaeDebug() == 0);
    }

    public Command AlgaeIn() {
        return run(() -> algaeIn()).until(() -> getAlgaeDebug() == 0);
    }
}