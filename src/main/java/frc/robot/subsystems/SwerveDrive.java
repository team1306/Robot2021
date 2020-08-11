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
  // (speed motor ID, angle motor ID)
  SwerveWheel frontLeft = new SwerveWheel(1, 2);
  SwerveWheel frontRight = new SwerveWheel(1, 2);
  SwerveWheel backLeft = new SwerveWheel(1, 2);
  SwerveWheel backRight = new SwerveWheel(1, 2);

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

  }

  //creates module states and assigns them to the specific wheels
  public void swerveDrive(double x1, double y1, double turn) {
    chassisSpeeds = new ChassisSpeeds(x1, y1, turn);

    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Front left module state
    SwerveModuleState frontLeftState = moduleStates[0];
    frontLeft.convert(frontLeftState);

    // Front right module state
    SwerveModuleState frontRightState = moduleStates[1];
    frontRight.convert(frontRightState);

    // Back left module state
    SwerveModuleState backLeftState = moduleStates[2];
    backLeft.convert(backLeftState);

    // Back right module state
    SwerveModuleState backRightState = moduleStates[3];
    backRight.convert(backRightState);
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