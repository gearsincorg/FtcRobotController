package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

public enum LEDMode {
    ALLIANCE_RED(GoBildaPrismDriver.Artboard.ARTBOARD_0),
    ALLIANCE_BLUE(GoBildaPrismDriver.Artboard.ARTBOARD_1),
    INTAKE_FRONT(GoBildaPrismDriver.Artboard.ARTBOARD_2),
    INTAKE_BACK(GoBildaPrismDriver.Artboard.ARTBOARD_3),
    SHOOTER_READY(GoBildaPrismDriver.Artboard.ARTBOARD_4),
    SHOOTER_NOT_READY(GoBildaPrismDriver.Artboard.ARTBOARD_5),
    INTAKE_JAMMED(GoBildaPrismDriver.Artboard.ARTBOARD_6);

    GoBildaPrismDriver.Artboard artboardValue;

    LEDMode(GoBildaPrismDriver.Artboard artboardValue){
        this.artboardValue = artboardValue;
    }

}
