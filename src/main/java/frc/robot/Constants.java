package frc.robot;


import edu.wpi.first.wpilibj.util.Units;

/**
 * A class for holding constant values in a single editable spot. What goes in
 * this file: -Robot Motorcontroller ports and IDS -Robot phsyical attributes
 * -Field attributes
 * 
 * Things that don't go in this file: -Subsystem-specific PID values
 * -Subsystem-specific sensor thresholds
 */
public final class Constants {

    // motors and encoders on the robot
    public static final int K_DRIVE_FRONT_LEFT_ID = 1;
    public static final int K_DRIVE_BACK_LEFT_ID = 2;
    public static final int K_DRIVE_FRONT_RIGHT_ID = 3;
    public static final int K_DRIVE_BACK_RIGHT_ID = 4;

    public static final int K_TURN_FRONT_LEFT_ID = 5;
    public static final int K_TURN_BACK_LEFT_ID = 6;
    public static final int K_TURN_FRONT_RIGHT_ID = 7;
    public static final int K_TURN_BACK_RIGHT_ID = 8;

    public static final int K_ENCODER_FRONT_LEFT_ID = 9;
    public static final int K_ENCODER_BACK_LEFT_ID = 10;
    public static final int K_ENCODER_FRONT_RIGHT_ID = 11;
    public static final int K_ENCODER_BACK_RIGHT_ID = 12;

    public static final int K_INTAKE = 12;

    public static final boolean DIRECTION_FORWARD = true;

    public static final int UNITS_PER_ROTATION = 4096;

    // robot information
    public static final double K_TRACK_WIDTH_METERS = 0.7175;
    public static final double K_WHEEL_RADIUS_INCHES = 3.75; //pnuematic, so ish
    public static final double K_WHEEL_RADIUS_METERS = Units.inchesToMeters(K_WHEEL_RADIUS_INCHES);

    public static final double MAX_VOLTS = 24;

    public static final double FASTEST_SPEED_METERS = 4.8768;

    

    /**
     * meters
     */
    public static final double WHEEL_DISTANCE_TO_CENTER = 0.50289434 / 2; // meters
    /**
     * radians per second!!!
     */
    public static final double FASTEST_ANGULAR_VELOCITY = (2 * Math.PI) * (WHEEL_DISTANCE_TO_CENTER * 2 * Math.PI) / FASTEST_SPEED_METERS;

    //wheel base is y
    //track front is width of front of the robot and the x value
    //track back is width of the back of the robot and the other x value
    public static final double ROBOT_WHEELBASE  = 1;
    public static final double ROBOT_TRACK_FRONT = 1;
    public static final double ROBOT_TRACK_BACK = 1;

    // autonomous constants
    public static final double A_kRamseteB = 2.0;
    public static final double A_kRamseteZeta = 0.7;
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 1;
    public static final double kaVoltSecondsSquaredPerMeter = 1;
}
