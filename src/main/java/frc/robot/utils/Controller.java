package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class Controller {
    // Controller Map:
    public static final int AXIS_RTRIGGER = 3;
    public static final int AXIS_LTRIGGER = 2;
    public static final int BUTTON_LTRIGGER = 5;
    public static final int BUTTON_RTRIGGER = 6;
    public static final int BUTTON_RBUMPER = BUTTON_RTRIGGER;
    public static final int BUTTON_LBUMPER = BUTTON_LTRIGGER;
    public static final int AXIS_LY = 1;
    public static final int AXIS_LX = 0;
    public static final int AXIS_RY = 5;
    public static final int AXIS_RX = 4;
    public static final int BUTTON_START = 8;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;

    public static final int PRIMARY = 0;
    public static final int SECONDARY = 1;

    public static Joystick primaryJoystick = null;
    public static Joystick secondaryJoystick = null;

    public static void init() {
        primaryJoystick = new Joystick(PRIMARY);
        secondaryJoystick = new Joystick(SECONDARY);
    }

    public static UserAnalog simpleAxis(int player, int axis) {
        Joystick joystick;
        if (player == PRIMARY) {
            joystick = primaryJoystick;
        } else if (player == SECONDARY) {
            joystick = secondaryJoystick;
        } else {
            System.err.println("ERROR: Invalid Player Controller requested");
            return () -> {
                return 0;
            };
        }
        return () -> {
            return joystick.getRawAxis(axis);
        };
    }

    public static UserDigital simpleButton(int player, int button){
        Joystick joystick;
        if (player == PRIMARY) {
            joystick = primaryJoystick;
        } else if (player == SECONDARY) {
            joystick = secondaryJoystick;
        } else {
            System.err.println("ERROR: Invalid Player Controller requested");
            return () -> {
                return false;
            };
        }
        return () -> {
            return joystick.getRawButton(button);
        };
    }
}
