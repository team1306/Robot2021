/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // CAN ids
    public static final int K_DRIVE_LEFT_FRONT_ID = 01;
    public static final int K_DRIVE_LEFT_BACK_ID = 02;

    public static final int K_DRIVE_RIGHT_FRONT_ID = 03;
    public static final int K_DRIVE_RIGHT_BACK_ID = 04;

    public static final int K_SHOOTER_FlYWHEEL_ID = 05;

    // PWM Ports
    public static final int K_SHOOTER_KICKER_SPARK=0;

    // DIO Ports
    public static final int K_DIST_SWITCH = 0;

    // Analog Ports
    public static final int K_DIST_SENSOR = 0;

}
