/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  // (speed motor ID, angle motor ID)
  SwerveWheel frontLeft = new SwerveWheel(Constants.K_DRIVE_LEFT_FRONT_ID, Constants.K_TURN_LEFT_FRONT_ID, Constants.K_ENCODER_LEFT_FRONT_ID);
  SwerveWheel frontRight = new SwerveWheel(Constants.K_DRIVE_RIGHT_FRONT_ID, Constants.K_TURN_RIGHT_FRONT_ID, Constants.K_ENCODER_RIGHT_FRONT_ID);
  SwerveWheel backLeft = new SwerveWheel(Constants.K_DRIVE_LEFT_BACK_ID, Constants.K_TURN_LEFT_BACK_ID, Constants.K_ENCODER_LEFT_BACK_ID);
  SwerveWheel backRight = new SwerveWheel(Constants.K_DRIVE_RIGHT_BACK_ID, Constants.K_TURN_RIGHT_BACK_ID, Constants.K_ENCODER_RIGHT_BACK_ID);

  Translation2d frontLeftWheel = new Translation2d(Constants.ROBOT_TRACK_FRONT, Constants.ROBOT_WHEELBASE / 2);
  Translation2d frontRightWheel = new Translation2d(Constants.ROBOT_TRACK_FRONT, -Constants.ROBOT_WHEELBASE / 2);
  Translation2d backLeftWheel = new Translation2d(-Constants.ROBOT_TRACK_BACK, Constants.ROBOT_WHEELBASE / 2);
  Translation2d backRightWheel = new Translation2d(-Constants.ROBOT_TRACK_BACK, -Constants.ROBOT_WHEELBASE / 2);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftWheel, frontRightWheel, backLeftWheel,
      backRightWheel);

  private ChassisSpeeds chassisSpeeds;

  /**
   * TODO Create default SwerveDrive constructor.
   */
  public SwerveDrive() {

  }

  /**
   * Creates four new SwerveModuleStates and assigns them to their respective wheels
   * @param x1 x-coordinate movement
   * @param y1 y-coordinate movement
   * @param turn rotation of the wheels
   */
  public void driveTrain(double x1, double y1, double turn) {
    resetEncoders();
    
    chassisSpeeds = new ChassisSpeeds(x1, y1, turn);

    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    //TODO: find max speed of robot wheels
    //SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);

    // Front left module state
    SwerveModuleState frontLeftState = moduleStates[0];
    frontLeft.drive(frontLeftState);

    // Front right module state
    SwerveModuleState frontRightState = moduleStates[1];
    frontRight.drive(frontRightState);

    // Back left module state
    SwerveModuleState backLeftState = moduleStates[2];
    backLeft.drive(backLeftState);

    // Back right module state
    SwerveModuleState backRightState = moduleStates[3];
    backRight.drive(backRightState);
  }

  /**
   * Resets encoder values to zero
   */
  public void resetEncoders() {
    //frontRight.resetEncoder();
    //frontLeft.resetEncoder();
    //backRight.resetEncoder();
    //backLeft.resetEncoder(); // different backLeft(other ones got commented out)
  }

  /**
   * Test method which returns value of the front right encoder
   * @return position of front right encoder
   */
  public double getFrontRightEnc() {
    return frontRight.getPosition();
  }
}