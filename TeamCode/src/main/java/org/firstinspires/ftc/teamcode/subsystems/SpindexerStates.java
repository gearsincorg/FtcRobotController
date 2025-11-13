package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.auxtools.StateBase;

public enum SpindexerStates implements StateBase {
    INIT,
    HOMING,
    HOME,
    INTAKING,
    QUEUEING,
    READY_TO_SHOOT,
    COCKING_SHOT,
    SHOOTING
}
