// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.IO;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
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
    public DifferentialDrivetrainSim simRobot;
    DriveTrainSimulationConfig driveSimConfig;
    SwerveDriveSimulation driveSim;

    public DrivetrainIO_SIM() {
        leftPid = new PIDController(0.9, 0, 0);
        rightPid = new PIDController(0.9, 0, 0);
        simRobot = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            8.46,
            0.8946636991,
            DesignConstants.robotMass,
            DesignConstants.wheelRadius,
            DesignConstants.robotWidth,
            null);
    }

    @Override
    public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        leftPid.setSetpoint(wheelSpeeds.leftMetersPerSecond);
        rightPid.setSetpoint(wheelSpeeds.rightMetersPerSecond);
    }

    @Override
    public void updateInputs(driveDataAutoLogged data) {  
        simRobot.update(0.020);      
        double voltsLeft = leftPid.calculate(simRobot.getLeftVelocityMetersPerSecond());
        double voltsRight = rightPid.calculate(simRobot.getRightVelocityMetersPerSecond());
        simRobot.setInputs(voltsLeft, voltsRight);
        
        // Getting Left Side Data
        data.ampsLeft = simRobot.getLeftCurrentDrawAmps();
        data.voltsLeft = voltsLeft;
        data.positionLeft = simRobot.getLeftPositionMeters();
        data.velocityLeft = simRobot.getLeftVelocityMetersPerSecond();

        // Getting Right Side Data
        data.ampsRight = simRobot.getRightCurrentDrawAmps();
        data.voltsRight = voltsRight;
        data.positionRight = simRobot.getRightPositionMeters();
        data.velocityRight = simRobot.getRightVelocityMetersPerSecond();

        data.currentPosition = simRobot.getPose();
    }

    @Override
    public void periodic() {
        
    }
}
