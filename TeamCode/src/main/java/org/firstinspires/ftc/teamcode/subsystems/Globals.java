package org.firstinspires.ftc.teamcode.subsystems;

public class Globals {

    /**
     * Match constants.
     */
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.RED;
    public static boolean IS_AUTO = false;
    public static boolean PURPLE_PIXEL_ON_RIGHT = false;

    /**
     * Robot state constants.
     */
    public static boolean ARM_HAS_HOMED = false;
    public static boolean LEFT_GRABBER_CLOSED = false;
    public static boolean RIGHT_GRABBER_CLOSED = false;

    public static ManipulatorState ARM_STATE = ManipulatorState.UNKNOWN;
    public static ManipulatorWristState WRIST_STATE = ManipulatorWristState.UNKNOWN;

    public static double LAST_HEADING = 0;
}