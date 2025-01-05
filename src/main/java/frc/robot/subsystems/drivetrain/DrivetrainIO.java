// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/** Add your docs here. */
public interface DrivetrainIO {
    @AutoLog
    public class driveData {
        public double voltsLeft = 0.0;
        public double voltsRight = 0.0;
        public double ampsLeft = 0.0;
        public double ampsRight = 0.0;
        public double positionLeft = 0.0;
        public double positionRight = 0.0;
        public double velocityLeft = 0.0; // Velocity Left Meters per a second
        public double velocityRight = 0.0; // Velocity Right Meters per a second
        public Pose2d currentPosition = new Pose2d();
    }
    public abstract void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds);
    public abstract void updateInputs(driveDataAutoLogged data);
    public abstract void periodic();
}
