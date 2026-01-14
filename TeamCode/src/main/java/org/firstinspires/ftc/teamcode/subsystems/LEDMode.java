package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

public enum LEDMode {
    POWER_UP(GoBildaPrismDriver.Artboard.ARTBOARD_0),
    ALLIANCE_RED(GoBildaPrismDriver.Artboard.ARTBOARD_1),
    ALLIANCE_BLUE(GoBildaPrismDriver.Artboard.ARTBOARD_2),
    INTAKE_FRONT(GoBildaPrismDriver.Artboard.ARTBOARD_3),
    INTAKE_BACK(GoBildaPrismDriver.Artboard.ARTBOARD_4),
    SHOOTER_READY(GoBildaPrismDriver.Artboard.ARTBOARD_5),
    SHOOTER_NOT_READY(GoBildaPrismDriver.Artboard.ARTBOARD_6),
    INTAKE_JAMMED(GoBildaPrismDriver.Artboard.ARTBOARD_7);

    GoBildaPrismDriver.Artboard artboardValue;
    LEDMode(GoBildaPrismDriver.Artboard artboardValue){
        this.artboardValue = artboardValue;
    }
}

