/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.UserAnalog;

/**
 * Command to drive the robot based on controller input.
 */
public class DriveCommand extends Command {

  UserAnalog right;
  UserAnalog left;
  DriveTrain driveTrain;
  private final boolean isArcade = true;

  public DriveCommand(DriveTrain driveTrain, UserAnalog right, UserAnalog left) {
    // Use requires() here to declare subsystem dependencies
    this.right = right;
    this.left = left;
    this.driveTrain = driveTrain;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Depending on the driver's preference, pass joystick values to drivetrain.
      driveTrain.drive(left.get(), right.get());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}