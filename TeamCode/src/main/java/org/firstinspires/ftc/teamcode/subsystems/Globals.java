package org.firstinspires.ftc.teamcode.subsystems;

public class Globals {

    /**
     * Match constants.
     */
    public static Side ALLIANCE = Side.RED;
    public static Side START_POSITION = Side.BACK;
    public static boolean IS_AUTO = false;

    /**
     * Robot state constants.
     */
    public static boolean ARM_HAS_HOMED = false;
    public static boolean LEFT_GRABBER_CLOSED = false;
    public static boolean RIGHT_GRABBER_CLOSED = false;

    public static ManipulatorState ARM_STATE = ManipulatorState.UNKNOWN;
    public static WristState WRIST_STATE = WristState.UNKNOWN;

    public static double LAST_HEADING = 0;
}