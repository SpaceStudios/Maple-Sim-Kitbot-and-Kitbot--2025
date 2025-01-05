// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.drivetrain.IO.DrivetrainIO_SIM;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  DifferentialDriveKinematics kinematics;
  DrivetrainIO io;
  driveDataAutoLogged data;
  public Drivetrain() {
    kinematics = new DifferentialDriveKinematics(0.5);
    data = new driveDataAutoLogged();
    switch (RobotConstants.currentStatus) {
      case REAL:

        break;
      case SIM:
        io = new DrivetrainIO_SIM();
        break;
    }
  }

  public void DriveBasedOnSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    io.driveWheelSpeeds(wheelSpeeds);
    Logger.recordOutput("Drive Train/Speeds", wheelSpeeds);
  }

  @Override
  public void periodic() {
    io.updateInputs(data);
    Logger.processInputs("Drive Train", data);
    Logger.recordOutput("Vision Pose Estimated", io.getPose());
  }
}
