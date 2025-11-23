package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;

public class Globals {
    /**
     * Match constants.
     */
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.RED;

    /**
     * Robot state constants.
     */
    public static Pose2d    LAST_POSE      = new Pose2d(0,0,0);
    public static boolean   IS_AUTO        = false;
    public static int       OCTO_ERRORS    = 0;
    public static boolean   FORWARD_MOTION = true;
    public static boolean   AT_SPEED       = false;

    public static RobotStates       ROBOT_STATE     = RobotStates.INACTIVE;
    public static SpindexerStates   SPINDEXER_STATE = SpindexerStates.INIT;
    public static TurretStates      TURRET_STATE    = TurretStates.INIT;

}

