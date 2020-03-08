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
    public static final int K_DRIVE_LEFT_FRONT_ID = 1;
    public static final int K_DRIVE_LEFT_MIDDLE_ID = 2;
    public static final int K_DRIVE_LEFT_BACK_ID = 3;

    public static final int K_DRIVE_RIGHT_FRONT_ID = 4;
    public static final int K_DRIVE_RIGHT_MIDDLE_ID = 5;
    public static final int K_DRIVE_RIGHT_BACK_ID = 6;

    public static final int K_SHOOTER_FlYWHEEL_ID = 7;

    public static final int K_SPINNER_MOTOR_ID = 8;

    public static final int K_SHOOTER_KICKER_ID = 9;
    public static final int K_INTAKE_AXEL_RIGHT_ID = 10;
    public static final int K_INTAKE_AXEL_LEFT_ID = 11;
    public static final int K_INTAKE_INDEXER_ID = 12;
    public static final int K_CLIMBER_MOTOR_ID = 13;

    // PWM Ports
    public static final int K_VISION_STATUS_LEDS_PORT = 0;

    // PCM ports
    public static final int K_SHOOTER_HOOD_UP_SOLENOID = 5;
    public static final int K_SHOOTER_HOOD_DWN_SOLENOID =4;
    public static final int K_DRIVE_SHIFT_LOW = 1;
    public static final int K_DRIVE_SHIFT_HIGH = 0;
    public static final int K_INTAKE_SOLENOID_UP = 2;
    public static final int K_INTAKE_SOLENOID_DOWN = 3;

    // DIO Ports
    public static final int K_INTAKE_INDEX_SWITCH_TOP = 0;
    public static final int K_INTAKE_INDEX_SWITCH_BOTTOM = 1;

    // Analog Ports
    public static final int K_DIST_SENSOR = 0;

    // Robot information -TODO
    public static final int K_TRACK_WIDTH_METERS = 1;
    public static final double K_WHEEL_RADIUS_INCHES = 3;

    public static final double K_SHOOTER_RADIUS_INCHES = 3;
    public static final double K_ANGLE_SHORT_DEGREES = 55;// all angles measured from the horizontal
    public static final double K_ANGLE_LONG_DEGREES = 35;
    public static final double K_SHOOTER_HEIGHT_FT = 3.5;

    public static final int K_NUM_LEDS = 28;

    // field information
    public static final double K_TARGET_HEIGHT_FT = 8.15;
}
