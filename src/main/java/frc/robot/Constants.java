package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * A class for holding constant values in a single editable spot. What goes in this file: -Robot
 * Motorcontroller ports and IDS -Robot phsyical attributes -Field attributes
 * 
 * Things that don't go in this file: -Subsystem-specific PID values -Subsystem-specific sensor
 * thresholds
 */
public final class Constants {

	public static final int K_INTAKE = 13;

	public static final boolean DIRECTION_FORWARD = true;

	public static final int UNITS_PER_ROTATION = 4096;

	// robot information
	public static final double K_TRACK_WIDTH_METERS = 0.7175;
	public static final double K_WHEEL_RADIUS_INCHES = 3.75; // pnuematic, so ish
	public static final double K_WHEEL_RADIUS_METERS = Units.inchesToMeters(
		K_WHEEL_RADIUS_INCHES
	);
	public static final double K_WHEEL_CIRCUMFERENCE_METERS = 2
		* Math.PI
		* K_WHEEL_RADIUS_METERS;

	// modified
	public static final double ENCODER_TICKS_TO_DEGREES = 360.0 / 4096.0;
	public static final double DEGREES_TO_ENCODER_TICKS = 2048.0 / 360.0;

	// fastest speed percent output
	public static final double FASTEST_SPEED_METERS = 0.5;

	public static final double SENSOR_UNIT_TO_DEG = 360 / (12.8 * 2048);

	public static final double GEAR_RATIO = 12.8;

	// wheel base is y
	// track front is width of front of the robot and the x value
	// track back is width of the back of the robot and the other x value
	public static final double ROBOT_DISTANCE_BETWEEN_WHEELS = (16.0 / 36.0);
	/**
	 * meters
	 */
	public static final double WHEEL_DISTANCE_TO_CENTER = Math.sqrt(
		ROBOT_DISTANCE_BETWEEN_WHEELS * ROBOT_DISTANCE_BETWEEN_WHEELS * 2
	);
	/**
	 * radians per second!!!
	 */
	public static final double FASTEST_ANGULAR_VELOCITY = FASTEST_SPEED_METERS
		* 2.0
		* Math.PI
		/ (WHEEL_DISTANCE_TO_CENTER * 2.0 * Math.PI * 2);

	// motors and encoders on the robot
	// if drive start at 1, if turn start at 5, if encoder start at 9
	// if on left +1 points
	// if on back + 2 points
	public static final int K_DRIVE_FRONT_RIGHT_ID = 1;
	public static final int K_DRIVE_FRONT_LEFT_ID = 2;
	public static final int K_DRIVE_BACK_RIGHT_ID = 3;
	public static final int K_DRIVE_BACK_LEFT_ID = 4;

	public static final int K_TURN_FRONT_RIGHT_ID = 5;
	public static final int K_TURN_FRONT_LEFT_ID = 6;
	public static final int K_TURN_BACK_RIGHT_ID = 7;
	public static final int K_TURN_BACK_LEFT_ID = 8;

	public static final int K_ENCODER_FRONT_RIGHT_ID = 9;
	public static final int K_ENCODER_FRONT_LEFT_ID = 10;
	public static final int K_ENCODER_BACK_RIGHT_ID = 11;
	public static final int K_ENCODER_BACK_LEFT_ID = 12;

	public static final double K_FRONT_RIGHT_OFFSET = 0;// (2680) * ENCODER_TICKS_TO_DEGREES ;
	public static final double K_FRONT_LEFT_OFFSET = 0;// (1500) * ENCODER_TICKS_TO_DEGREES;
	public static final double K_BACK_RIGHT_OFFSET = 0;// (-707-144) * ENCODER_TICKS_TO_DEGREES;
	public static final double K_BACK_LEFT_OFFSET = 0;// (-2545) * ENCODER_TICKS_TO_DEGREES;

	// front right 2 front left -20 back right -105 back left -232
}
