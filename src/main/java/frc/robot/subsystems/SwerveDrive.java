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

/**
 * The SwerveDrive class uses four SwerveWheel objects which make up the
 * drivetrain. This class is responsible for the math that keeps track of how
 * the swerve drive moves
 */
public class SwerveDrive extends SubsystemBase {
    // (speed motor ID, angle motor ID)
    public SwerveWheel frontLeft = new SwerveWheel(Constants.K_DRIVE_FRONT_LEFT_ID, Constants.K_TURN_FRONT_LEFT_ID, Constants.K_ENCODER_FRONT_LEFT_ID);
    public SwerveWheel frontRight = new SwerveWheel(Constants.K_DRIVE_FRONT_RIGHT_ID, Constants.K_TURN_FRONT_RIGHT_ID, Constants.K_ENCODER_FRONT_RIGHT_ID);
    public SwerveWheel backLeft = new SwerveWheel(Constants.K_DRIVE_BACK_LEFT_ID, Constants.K_TURN_BACK_LEFT_ID, Constants.K_ENCODER_BACK_LEFT_ID);
    public SwerveWheel backRight = new SwerveWheel(Constants.K_DRIVE_BACK_RIGHT_ID, Constants.K_TURN_BACK_RIGHT_ID, Constants.K_ENCODER_BACK_RIGHT_ID);

    Translation2d frontLeftWheel = new Translation2d(Constants.ROBOT_TRACK_FRONT, Constants.ROBOT_WHEELBASE / 2);
    Translation2d frontRightWheel = new Translation2d(Constants.ROBOT_TRACK_FRONT, -Constants.ROBOT_WHEELBASE / 2);
    Translation2d backLeftWheel = new Translation2d(-Constants.ROBOT_TRACK_BACK, Constants.ROBOT_WHEELBASE / 2);
    Translation2d backRightWheel = new Translation2d(-Constants.ROBOT_TRACK_BACK, -Constants.ROBOT_WHEELBASE / 2);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel);
    private ChassisSpeeds chassisSpeeds;

    /**
     * Nothing needs to be done in the default constructor
     */
    public SwerveDrive() {
    }

    /**
     * Creates four new SwerveModuleStates and assigns them to their respective
     * wheels
     * 
     * @param x1    x-coordinate movement in meters per second
     * @param y1    y-coordinate movement in meters per second
     * @param turn  rotation of the wheels in radians per second
     */
    public void driveTrain(double x1, double y1, double turn) {
        chassisSpeeds = new ChassisSpeeds(x1, y1, turn);

        // convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.FASTEST_SPEED_METERS);

        SwerveModuleState frontLeftState = moduleStates[0];
        frontLeft.drive(frontLeftState);
        // frontLeft.sketchyDrive(frontLeftState);

        SwerveModuleState frontRightState = moduleStates[1];
        frontRight.drive(frontRightState);
        // frontRight.sketchyDrive(frontRightState);

        SwerveModuleState backLeftState = moduleStates[2];
        backLeft.drive(backLeftState);
        // backLeft.sketchyDrive(backLeftState);

        SwerveModuleState backRightState = moduleStates[3];
        backRight.drive(backRightState);
        // backRight.sketchyDrive(backRightState);
    }
}