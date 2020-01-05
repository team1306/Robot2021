/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.UserAnalog;

/**
 * An example command that uses an example subsystem.
 */
public class UserDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain driveTrain;
  private final UserAnalog driveLeft;
  private final UserAnalog driveRight;

  /**
   * @param driveTrain - the robot's DriveTrain instance
   * @param driveRight - the user input llambda for getting the right velocity
   * @param driveLeft  - the user input llambda for getting the left velocity
   */
  public UserDrive(DriveTrain driveTrain, UserAnalog driveRight, UserAnalog driveLeft) {
    this.driveTrain = driveTrain;
    this.driveLeft = driveLeft;
    this.driveRight = driveRight;
    addRequirements(driveTrain);
    driveTrain.setDefaultCommand(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.tankDrive(driveRight.get(), driveLeft.get());
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
