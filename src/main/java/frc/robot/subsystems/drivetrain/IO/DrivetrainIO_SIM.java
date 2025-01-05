// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.IO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants.DesignConstants;
import frc.robot.subsystems.drivetrain.DrivetrainIO;
import frc.robot.subsystems.drivetrain.driveDataAutoLogged;

/** Add your docs here. */
public class DrivetrainIO_SIM implements DrivetrainIO {
    PIDController leftPid;
    PIDController rightPid;
    DifferentialDrivetrainSim simrig;
    

    public DrivetrainIO_SIM() {
        leftPid = new PIDController(0.9, 0, 0);
        rightPid = new PIDController(0.9, 0, 0);
        simrig = new DifferentialDrivetrainSim(DCMotor.getNEO(2),1/8.46, 0.8946636991, DesignConstants.robotMass, DesignConstants.wheelRadius, DesignConstants.robotWidth, null);
    }

    @Override
    public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        leftPid.setSetpoint(wheelSpeeds.leftMetersPerSecond);
        rightPid.setSetpoint(wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public void updateInputs(driveDataAutoLogged data) {        
        double voltsLeft = leftPid.calculate(simrig.getLeftVelocityMetersPerSecond());
        double voltsRight = rightPid.calculate(simrig.getRightVelocityMetersPerSecond());
        simrig.setInputs(voltsLeft, voltsRight);
        
        // Getting Left Side Data
        data.ampsLeft = simrig.getLeftCurrentDrawAmps();
        data.voltsLeft = voltsLeft;
        data.positionLeft = simrig.getLeftPositionMeters();
        data.velocityLeft = simrig.getLeftVelocityMetersPerSecond();

        // Getting Right Side Data
        data.ampsRight = simrig.getRightCurrentDrawAmps();
        data.voltsRight = voltsRight;
        data.positionRight = simrig.getRightPositionMeters();
        data.velocityRight = simrig.getRightVelocityMetersPerSecond();
        data.currentPosition = simrig.getPose();
    }

    @Override
    public void periodic() {
        
    }
}
