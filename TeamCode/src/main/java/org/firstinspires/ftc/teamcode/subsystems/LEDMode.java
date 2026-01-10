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

/*
    Artboard       Front 12 (0-11)    MID 6 (12-17)      Back (18-29)
        0           Yellow Droid         White (Sat 0)   Yellow Droid   Hue  32,  Saturation 1, Brightness 100, speed 0.1)
        1           Red Snake            White (Sat 0)   Red Snake      Hue   0,  Saturation 1, Brightness 100, speed 0.4)
        2           Blue Snake           White (Sat 0)   Blue Snake     Hue 240,  Saturation 1, Brightness 100, speed 0.4)
 */