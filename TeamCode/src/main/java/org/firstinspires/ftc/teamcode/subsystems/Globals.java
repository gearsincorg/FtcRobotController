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
    public static Pose2d LAST_POSE = new Pose2d(0,0,0);
    public static boolean IS_AUTO       = false;
    public static boolean ARM_HOMED     = false;
    public static boolean LIFT_HOMED    = false;
    public static boolean SLIDE_HOMED   = false;
    public static int     OCTO_ERRORS   = 0;
    public static boolean RC_END        = false;
    public static boolean RC_SWEEP      = false;
    public static boolean DID_NOT_SWEEP_SAMPLE = false;
    public static boolean SAFE_TO_TRANSFER = false;


    public static double  DRIVE_AXIAL   = 0;    // used by the Lift subsystem to monitor the motor drive.
    public static double  DRIVE_LATERAL = 0;    // used by the Lift subsystem to monitor the motor drive.
    public static double  DRIVE_YAW     = 0;    // used by the Lift subsystem to monitor the motor drive.
}