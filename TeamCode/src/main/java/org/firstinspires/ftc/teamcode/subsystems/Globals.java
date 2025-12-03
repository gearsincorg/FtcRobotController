package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

public class Globals {
    /**
     * Match constants.
     */
    public static AllianceColor ALLIANCE_COLOR = AllianceColor.BLUE;

    /**
     * Robot state constants.
     */
    public static Pose2d    LAST_POSE        = new Pose2d(0,0,0);
    public static boolean   IS_AUTO          = false;
    public static int       OCTO_ERRORS      = 0;
    public static boolean   TURRET_HAS_HOMED = false;
    public static boolean   FORWARD_MOTION   = true;

    public static boolean   SHOOTER_AT_SPEED = false;
    public static boolean   TURRET_ON_TARGET = false;

    public static RobotStates       ROBOT_STATE     = RobotStates.INACTIVE;
    public static SpindexerStates   SPINDEXER_STATE = SpindexerStates.INIT;
    public static TurretStates      TURRET_STATE    = TurretStates.INIT;

    public static Action actionSetRobotState(RobotStates state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ROBOT_STATE = state;
                return false;
            }
        };
    }
}

