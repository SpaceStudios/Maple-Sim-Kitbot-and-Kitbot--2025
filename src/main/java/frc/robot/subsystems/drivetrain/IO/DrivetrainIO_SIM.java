// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.IO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants.DesignConstants;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.driveDataAutoLogged;

/** Add your docs here. */
public class DrivetrainIO_SIM implements DrivetrainIO {
    PIDController leftPid;
    PIDController rightPid;
    
    

    public DrivetrainIO_SIM() {
        leftPid = new PIDController(0.9, 0, 0);
        rightPid = new PIDController(0.9, 0, 0);
    }

    @Override
    public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        leftPid.setSetpoint(wheelSpeeds.leftMetersPerSecond/DesignConstants.wheelRadius);
        rightPid.setSetpoint(wheelSpeeds.rightMetersPerSecond/DesignConstants.wheelRadius);
    }

    @Override
    public void updateInputs(driveDataAutoLogged data) {
        
    }

    @Override
    public void periodic() {
        
    }
}
