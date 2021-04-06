/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.UserAnalog;

/**
 * Command to drive the robot based on controller input.
 */
public class DriveCommand extends CommandBase {

  UserAnalog right;
  UserAnalog forward;
  UserAnalog backward;
  DriveTrain driveTrain;
  private final boolean isArcade = true;

  public DriveCommand(DriveTrain driveTrain, UserAnalog turn, UserAnalog forward, UserAnalog backward) {
    // Use requires() here to declare subsystem dependencies
    System.out.println("Drive Command constructor is running.");
    this.right = turn;

    this.forward = forward;
    this.backward = backward;
    this.driveTrain = driveTrain;
    this.addRequirements(driveTrain);
    this.driveTrain.setDefaultCommand(this);

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    System.out.println("Drive Command execute is running.");

    //Depending on the driver's preference, pass joystick values to drivetrain.
      driveTrain.arcadeDrive(forward.get() - backward.get(), right.get(), true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }
}