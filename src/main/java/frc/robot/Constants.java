package frc.robot;

/**
 * A class for holding constant values in a single editable spot. What goes in
 * this file: -Robot Motorcontroller ports and IDS -Robot phsyical attributes
 * -Field attributes
 * 
 * Things that don't go in this file: -Subsystem-specific PID values
 * -Subsystem-specific sensor thresholds
 */
public final class Constants {

    // CAN ids
    public static final int K_DRIVE_LEFT_FRONT_ID = 01;
    public static final int K_DRIVE_LEFT_MIDDLE_ID = 02;
    public static final int K_DRIVE_LEFT_BACK_ID = 03;

    public static final int K_DRIVE_RIGHT_FRONT_ID = 04;
    public static final int K_DRIVE_RIGHT_MIDDLE_ID = 05;
    public static final int K_DRIVE_RIGHT_BACK_ID = 06;

    public static final int K_SHOOTER_FlYWHEEL_ID = 07;

    public static final int K_SPINNER_MOTOR_ID = 8;

    // PWM Ports
    public static final int K_SHOOTER_KICKER_SPARK = 0;
    public static final int K_INTAKE_AXEL_MAIN_SPARK = 1;
    public static final int K_INTAKE_AXEL_LEFT_SPARK = 2;
    public static final int K_INTAKE_INDEXER_SPARK = 3;
    public static final int K_VISION_STATUS_LEDS_PORT = 4;

    // PCM ports
    public static final int K_SHOOTER_HOOD_UP_SOLENOID = 0;
    public static final int K_SHOOTER_HOOD_DWN_SOLENOID = 1;
    public static final int K_DRIVE_SHIFT_RIGHT_FWD = 2;
    public static final int K_DRIVE_SHIFT_RIGHT_BKWD = 3;
    public static final int K_DRIVE_SHIFT_LEFT_FWD = 4;
    public static final int K_DRIVE_SHIFT_LEFT_BKWD = 5;

    // DIO Ports
    public static final int K_INTAKE_INDEX_SWITCH = 0;

    // Analog Ports
    public static final int K_DIST_SENSOR = 0;

    // Robot information -TODO
    public static final int K_TRACK_WIDTH_METERS = 1;
    public static final double K_WHEEL_RADIUS_INCHES = 3;

    public static final double K_SHOOTER_RADIUS_INCHES = 3;
    public static final double K_ANGLE_SHORT_DEGREES = 55;// all angles measured from the horizontal
    public static final double K_ANGLE_LONG_DEGREES = 35;
    public static final double K_SHOOTER_HEIGHT_FT = 3.5;

    public static final int K_NUM_LEDS = 15;

    // field information
    public static final double K_TARGET_HEIGHT_FT = 8.15;
}
