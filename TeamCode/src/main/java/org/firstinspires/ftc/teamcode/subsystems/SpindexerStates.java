package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.auxtools.StateBase;

public enum SpindexerStates implements StateBase {
    INIT,
    HOMING,
    HOME,
    INTAKING,
    FULL,
    QUEUEING,
    QUEUED,
    SHOOTING,
    RELOADING
}
