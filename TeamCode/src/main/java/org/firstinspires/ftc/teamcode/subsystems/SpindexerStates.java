package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.auxtools.StateBase;

public enum SpindexerStates implements StateBase {
    INIT,
    HOMING,
    HOME,
    STOPPED,
    INTAKING,
    FULL,
    LOADING_GREEN,
    LOADING_PURPLE,
    LOADING_NEXT,
    SHOOTING,
    RELOADING
}
