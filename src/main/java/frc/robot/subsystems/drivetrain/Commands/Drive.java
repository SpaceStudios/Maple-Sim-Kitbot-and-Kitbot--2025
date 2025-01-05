// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  DoubleSupplier Joystick1Y;
  DoubleSupplier Joystick2X;
  Drivetrain drivetrain;
  public Drive(DoubleSupplier joystick1Y, DoubleSupplier joystick2X, Drivetrain cDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    Joystick1Y = joystick1Y;
    Joystick2X = joystick2X;
    drivetrain = cDrivetrain;
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds setSpeeds = new ChassisSpeeds(Joystick1Y.getAsDouble()*RobotConstants.RobotMaxSpeed, 0.0, Joystick2X.getAsDouble()*RobotConstants.RobotMaxTurnSpeed);
    drivetrain.DriveBasedOnSpeeds(setSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
