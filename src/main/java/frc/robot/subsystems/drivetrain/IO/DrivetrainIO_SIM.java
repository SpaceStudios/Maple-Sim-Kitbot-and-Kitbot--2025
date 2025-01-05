// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.IO;

import java.io.IOException;
import java.util.Optional;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    VisionSystemSim visionSim;
    PhotonPoseEstimator poseEstimator;
    Pose2d previousPose;
    PhotonCameraSim cam1Sim;

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
            null
        );
        // Vision
        visionSim = new VisionSystemSim("vision");
        AprilTagFieldLayout fieldTags = null;
        try {
            fieldTags = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
            visionSim.addAprilTags(fieldTags);
        } catch (IOException e) {
            e.printStackTrace();
        }
        SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);
        PhotonCamera cam1 = new PhotonCamera("camera1");
        cam1Sim = new PhotonCameraSim(cam1, cameraProperties);
        Translation3d cam1Pos = new Translation3d(0.1, 0, 0);
        Rotation3d cam1Rot = new Rotation3d(0, Units.degreesToRadians(-15), 0);
        Transform3d cam1Transform = new Transform3d(cam1Pos, cam1Rot);
        visionSim.addCamera(cam1Sim, cam1Transform);
        poseEstimator = new PhotonPoseEstimator(fieldTags, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam1Transform);
        previousPose = new Pose2d();
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
        visionSim.update(simRobot.getPose());

        SmartDashboard.putData("Vision Sim Field", visionSim.getDebugField());
    }

    @Override
    public void setPose(Pose2d setPose) {
        simRobot.setPose(setPose);
        previousPose = setPose;
    }

    @Override
    public Pose2d getPose() {
        poseEstimator.setReferencePose(previousPose);
        Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(cam1Sim.getCamera().getLatestResult());
        if (estimatedRobotPose.isPresent()) {
            previousPose = estimatedRobotPose.get().estimatedPose.toPose2d();
            return estimatedRobotPose.get().estimatedPose.toPose2d();
        } else {
            return previousPose;
        }
    }
}
