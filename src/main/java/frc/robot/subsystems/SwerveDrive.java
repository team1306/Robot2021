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
//import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANSparkMax;

//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.ControlType;
//import com.revrobotics.EncoderType;
//import com.revrobotics.CANSparkMax.IdleMode;

/*import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;*/
import frc.robot.Constants;;

public class SwerveDrive extends SubsystemBase {

  // declare
  private final double width = 1;
  private final double length = 1;

  // private final swerveWheel frontRight; 
  // private final swerveWheel frontLeft;
  // private final swerveWheel backRight;
  // private final swerveWheel backLeft;

  // (speed motor ID, angle motor ID)
  swerveWheel frontLeft = new swerveWheel(1, 2);
  swerveWheel frontRight = new swerveWheel(1, 2);
  swerveWheel backLeft = new swerveWheel(1, 2);
  swerveWheel backRight = new swerveWheel(1, 2);

  Translation2d frontLeftWheel = new Translation2d(Constants.ROBOT_TRACK_FRONT, Constants.ROBOT_WHEELBASE / 2);
  Translation2d frontRightWheel = new Translation2d(Constants.ROBOT_TRACK_FRONT, -Constants.ROBOT_WHEELBASE / 2);
  Translation2d backLeftWheel = new Translation2d(-Constants.ROBOT_TRACK_BACK, Constants.ROBOT_WHEELBASE / 2);
  Translation2d backRightWheel = new Translation2d(-Constants.ROBOT_TRACK_BACK, -Constants.ROBOT_WHEELBASE / 2);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftWheel, frontRightWheel, backLeftWheel,
      backRightWheel);

  private ChassisSpeeds chassisSpeeds;

  /**
   * Creates a new ExampleSubsystem.
   */
  public SwerveDrive() {

    // frontRight = new swerveWheel(Constants.K_DRIVE_RIGHT_FRONT_ID, Constants.K_ANGLE_LEFT_FRONT_ID, 1);
    // frontLeft = new swerveWheel(Constants.K_DRIVE_LEFT_FRONT_ID, Constants.K_ANGLE_LEFT_FRONT_ID, 2);
    // backRight = new swerveWheel(Constants.K_DRIVE_RIGHT_BACK_ID, Constants.K_ANGLE_RIGHT_BACK_ID, 2);
    // backLeft = new swerveWheel(Constants.K_DRIVE_LEFT_BACK_ID, Constants.K_ANGLE_LEFT_BACK_ID, 1);
  }

  // public void swerveDrive(double x1, double y1, double turn) {
  // double r = Math.sqrt ((length * length) + (width * width));
  // y1 *= -1;

  // double a = x1 - turn * (length / r);
  // double b = x1 + turn * (length / r);
  // double c = y1 - turn * (width / r);
  // double d = y1 + turn * (width / r);

  // double backRightSpeed = Math.sqrt ((a * a) + (d * d));
  // double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
  // double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
  // double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

  // double backRightAngle = Math.atan2 (a, d) / Math.PI;
  // double backLeftAngle = Math.atan2 (a, c) / Math.PI;
  // double frontRightAngle = Math.atan2 (b, d) / Math.PI;
  // double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

  // frontRight.drive(frontRightSpeed, frontRightAngle);
  // frontLeft.drive(frontLeftSpeed, frontLeftAngle);
  // backRight.drive(backRightSpeed, backRightAngle);
  // backLeft.drive(backLeftSpeed, backLeftAngle);
  // }

  public void swerveDrive(double x1, double y1, double turn) {
    chassisSpeeds = new ChassisSpeeds(x1, y1, turn);

    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Front left module state
    SwerveModuleState frontLeftState = moduleStates[0];

    // frontLeft.convert() testing swerveWheel.convert method

    // Front right module state
    SwerveModuleState frontRightState = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeftState = moduleStates[2];

    // Back right module state
    SwerveModuleState backRightState = moduleStates[3];
  }

  public void resetEncoders() {
    frontRight.resetEncoder();
    frontLeft.resetEncoder();
    backRight.resetEncoder();
    backLeft.resetEncoder(); // different backLeft(other ones got commented out)
  }

  public double getFrontRightEnc() {
    return frontRight.getPosition();
  }
}