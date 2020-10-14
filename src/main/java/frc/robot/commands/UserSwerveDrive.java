/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.UserAnalog;


/**
 * An example command that uses an example subsystem.
 * TODO change UserDrive to create a SwerveDrive object
 */
public class UserSwerveDrive extends CommandBase {
  private final SwerveDrive m_swerveDrive;
  private final UserAnalog driveLeft;
  private final UserAnalog driveRight;
  private final UserAnalog driveTurn;

  //çhange names
  public UserSwerveDrive(SwerveDrive m_swerveDrive, UserAnalog driveRight, UserAnalog driveLeft, UserAnalog driveTurn) {
    this.m_swerveDrive = m_swerveDrive;
    this.driveLeft = driveLeft;
    this.driveRight = driveRight;
    this.driveTurn = driveTurn;
  }

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveDrive.swerveDrive(driveRight.get(), driveLeft.get(), driveTurn.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.swerveDrive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}