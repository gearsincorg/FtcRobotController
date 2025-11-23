package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.auxtools.StateBase;

public enum SpindexerStates implements StateBase {
    INIT,
    HOME,
    INTAKE_QUEUEING,
    INTAKING,
    INTAKE_HOLD,
    SHOT_QUEUEING,
    READY_TO_SHOOT,
    COCKING_SHOT,
    SHOOTING
}
