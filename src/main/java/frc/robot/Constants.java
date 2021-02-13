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

    // motors
    
    public static final int K_DRIVE_LEFT_FRONT_ID = 1;
    public static final int K_DRIVE_LEFT_BACK_ID = 2;
    public static final int K_DRIVE_RIGHT_FRONT_ID = 3;
    public static final int K_DRIVE_RIGHT_BACK_ID = 4;

    public static final int K_TURN_LEFT_FRONT_ID = 5;
    public static final int K_TURN_LEFT_BACK_ID = 6;
    public static final int K_TURN_RIGHT_FRONT_ID = 7;
    public static final int K_TURN_RIGHT_BACK_ID = 8;

    public static final int K_ENCODER_LEFT_FRONT_ID = 9;
    public static final int K_ENCODER_LEFT_BACK_ID = 10;
    public static final int K_ENCODER_RIGHT_FRONT_ID = 11;
    public static final int K_ENCODER_RIGHT_BACK_ID = 12;

    public static final int K_INTAKE = 12;

    // TODO Robot information
    public static final double K_TRACK_WIDTH_METERS = 0.7175;
    public static final double K_WHEEL_RADIUS_INCHES = 3.75;//pnuematic, so ish
    public static final double K_WHEEL_RADIUS_METERS = Units.inchesToMeters(K_WHEEL_RADIUS_INCHES);

    public static final double MAX_VOLTS = 24;
    public static final int ROTATIONS_ON_ENCODER = 4096;

    /**
     * meters
     */
    public static final double WHEEL_DISTANCE_TO_CENTER = 0.50289434 / 2; // meters

    //wheel base is y
    //track front is width of front of the robot and the x value
    //track back is width of the back of the robot and the other x value
    public static final double ROBOT_WHEELBASE  = 1;
    public static final double ROBOT_TRACK_FRONT = 1;
    public static final double ROBOT_TRACK_BACK = 1;
}
