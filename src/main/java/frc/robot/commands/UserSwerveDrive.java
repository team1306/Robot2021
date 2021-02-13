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
 */
public class UserSwerveDrive extends CommandBase {
  private final SwerveDrive m_swerveDrive;
  private final UserAnalog driveX;
  private final UserAnalog driveY;
  private final UserAnalog driveTurn;

  //çhange names
  /**
   * 
   * @param m_swerveDrive
   * @param driveY analog input for up/down movement
   * @param driveX analog input for left/right movement
   * @param driveTurn analog input for turning
   */
  public UserSwerveDrive(SwerveDrive m_swerveDrive, UserAnalog driveY, UserAnalog driveX, UserAnalog driveTurn) {
    this.m_swerveDrive = m_swerveDrive;
    this.driveX = driveX;
    this.driveY = driveY;
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
    m_swerveDrive.driveTrain(driveX.get(), driveY.get(), driveTurn.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.driveTrain(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}