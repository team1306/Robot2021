/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.UserDigital;

/**
 * The SwerveDrive class uses four SwerveWheel objects which make up the drivetrain. This class is
 * responsible for the math that keeps track of how the swerve drive moves
 */
public class SwerveDrive extends SubsystemBase {

	// (speed motor ID, angle motor ID)
	public SwerveWheel frontRight = new SwerveWheel(
		Constants.K_DRIVE_FRONT_RIGHT_ID, Constants.K_TURN_FRONT_RIGHT_ID,
		Constants.K_ENCODER_FRONT_RIGHT_ID
	);
	public SwerveWheel frontLeft = new SwerveWheel(
		Constants.K_DRIVE_FRONT_LEFT_ID, Constants.K_TURN_FRONT_LEFT_ID,
		Constants.K_ENCODER_FRONT_LEFT_ID
	);
	public SwerveWheel backRight = new SwerveWheel(
		Constants.K_DRIVE_BACK_RIGHT_ID, Constants.K_TURN_BACK_RIGHT_ID,
		Constants.K_ENCODER_BACK_RIGHT_ID
	);
	public SwerveWheel backLeft = new SwerveWheel(
		Constants.K_DRIVE_BACK_LEFT_ID, Constants.K_TURN_BACK_LEFT_ID,
		Constants.K_ENCODER_BACK_LEFT_ID
	);

	// center of rotation (middle of the robot)
	// change the [divided by two] to readjust robot center of rotation (currently also center of
	// mass because the robot is symmetrical)
	Translation2d frontLeftWheel = new Translation2d(
		-Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2,
		Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2
	);
	Translation2d frontRightWheel = new Translation2d(
		Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2,
		Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2
	);
	Translation2d backLeftWheel = new Translation2d(
		-Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2,
		-Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2
	);
	Translation2d backRightWheel = new Translation2d(
		Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2,
		-Constants.ROBOT_DISTANCE_BETWEEN_WHEELS / 2
	);

	public SwerveDriveOdometry odometry;
	/**
	 * TODO Gyro we use, navX Sensor; !!!remember to reset for optimal temperature by holding cal
	 * button for at least 10 secs, and then click reset button afterwards; if done properly,
	 * calibrate light blinks once
	 */
	public static AHRS gyro = new AHRS();
	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
		frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel
	);
	private ChassisSpeeds chassisSpeeds;

	private SwerveModuleState[] moduleStates;

	public final SwerveDriveOdometry sdo;
	private double time = System.currentTimeMillis() / 1000;

	/**
	 * Nothing needs to be done in the default constructor
	 */
	public SwerveDrive() {
		sdo = new SwerveDriveOdometry(
			kinematics, Rotation2d.fromDegrees(getYaw())
		);
	}

	/**
	 * returns yaw (rotation along the y-axis)
	 * 
	 * @return a double between (-180,180) that represents yaw
	 */
	public static double getYaw() {
		return gyro.getYaw();
	}

	/**
	 * Creates four new SwerveModuleStates and assigns them to their respective wheels
	 * 
	 * @param x    x-coordinate movement in meters per second
	 * @param y    y-coordinate movement in meters per second
	 * @param turn rotation of the wheels in radians per second
	 */
	public void driveTrain(double x, double y, double turn) {
		// Converts from the x-coord, y-coord and turns into an array of module states
		chassisSpeeds = new ChassisSpeeds(y, x, turn);
		ChassisSpeeds chassisSpeeds2 = new ChassisSpeeds(y, x, -turn);
		moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveModuleState[] modulesStates2 = kinematics.toSwerveModuleStates(
			chassisSpeeds2
		);

		// making sure module states have possible values
		SwerveDriveKinematics.normalizeWheelSpeeds(
			moduleStates,
			Constants.FASTEST_SPEED_METERS
		);
		SwerveDriveKinematics.normalizeWheelSpeeds(
			modulesStates2,
			Constants.FASTEST_SPEED_METERS
		);
		// Getting and assigning the module states to the wheels

		SwerveModuleState frontLeftState = modulesStates2[0];
		// frontLeft.drive(x,y,turn);
		frontLeft.drive(frontLeftState);

		SwerveModuleState frontRightState = moduleStates[1];
		// frontRight.drive(x,y,turn);
		frontRight.drive(frontRightState);

		SwerveModuleState backLeftState = moduleStates[2];
		// backLeft.drive(x,y,turn);
		backLeft.drive(backLeftState);

		SwerveModuleState backRightState = modulesStates2[3];
		// backRight.drive(x,y,turn);
		backRight.drive(backRightState);
		double newtime = System.currentTimeMillis() / 1000;
		sdo.updateWithTime(
			newtime - time,
			Rotation2d.fromDegrees(getYaw()),
			moduleStates
		);
		time = newtime;

		/*
		 * frontLeft.drive(FROn, FLOn, BROn, BLOn); frontRight.drive(FROn, FLOn, BROn, BLOn);
		 * backRight.drive(FROn, FLOn, BROn, BLOn); backLeft.drive(FROn, FLOn, BROn, BLOn);
		 */

		shuffleboard();
	}

	private int turn = 0;

	public void driveTrain(
		double speed,
		boolean x,
		boolean y,
		boolean a,
		boolean b
	) {
		if (x)
			turn = 270;
		else if (y)
			turn = 0;
		else if (a)
			turn = 180;
		else if (b)
			turn = 90;

		var state = new SwerveModuleState(speed, Rotation2d.fromDegrees(turn));

		frontLeft.drive(state);
		frontRight.drive(state);
		backLeft.drive(state);
		backRight.drive(state);
	}

	public double getAngle() {
		return 0;
	}

	public void driveTrain(double rotations, double angle) {
		frontLeft.drive(rotations, angle);
		frontRight.drive(rotations, angle);
		backLeft.drive(rotations, angle);
		backRight.drive(rotations, angle);
	}

	public void setModuleStates(SwerveModuleState[] states) {
		moduleStates = states;
	}

	private void shuffleboard() {
		/**
		 * as of 12/28: There are two problems with the robot! *three now* - The angle (gyro yaw)
		 * GIVEN that the robot is broken (inverted) the graph climbs up both when the robot is
		 * turning left and when the robot is turning right. - The robot is denoted Broken when at
		 * some point after the Yaw Reset, it just confuses backwards and forwards. The exact
		 * behavior being that it thinks the back of the robot (with the battery side) is forwards
		 * instead of the PDP side being forwards - Yaw does not match a tangent graph. even if it
		 * is very close.
		 */
		// TODO temporarily changed getYaw() to getAngle()
		SmartDashboard.putNumber("Gryo Yaw", gyro.getYaw());
		SmartDashboard.putNumber(
			"swervedriveodo X pos",
			sdo.getPoseMeters().getX()
		);
		SmartDashboard.putNumber(
			"swervedriveodo Y pos",
			sdo.getPoseMeters().getY()
		);
		// gyro.getAngle();

	}

	public void resetGyro(boolean A) {
		if (A)
			gyro.reset();
	}

	public void resetEncoders() {
		frontLeft.resetEncoder();
		backLeft.resetEncoder();
		backRight.resetEncoder();
		frontRight.resetEncoder();
	}

	public double getDegress() {
		return frontLeft.getRotation();
	}

	public double getGyroDisplacementX() {
		// Returns displacement of X axis in meters
		return gyro.getDisplacementX();
	}

	public double getGyroDisplacementY() {
		// Returns displacement of Y axis in meters
		return gyro.getDisplacementY();
	}

	public double getGyroDisplacementUseless() {
		// Returns displacement of Useless axis in meters
		return gyro.getDisplacementZ();
	}


}