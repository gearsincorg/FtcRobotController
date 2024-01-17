package org.firstinspires.ftc.teamcode.subsystems;

public class Globals {

    /**
     * Match constants.
     */
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.RED;
    public static boolean IS_AUTO = false;
    public static boolean PURPLE_PIXEL_ON_RIGHT = false;
    public static boolean PLACE_YELLOW_HIGH = false;

    /**
     * Robot state constants.
     */
    public static boolean ARM_HAS_HOMED = false;
    public static boolean HOLDING_LEFT_PIXEL = false;
    public static boolean HOLDING_RIGHT_PIXEL = false;

    public static ManipulatorState ARM_STATE = ManipulatorState.UNKNOWN;
    public static ManipulatorWristState WRIST_STATE = ManipulatorWristState.UNKNOWN;

    public static TeamPropLocation TEAM_PROP_LOCATION = TeamPropLocation.UNKNOWN;

    public static double LAST_HEADING = 0;
}