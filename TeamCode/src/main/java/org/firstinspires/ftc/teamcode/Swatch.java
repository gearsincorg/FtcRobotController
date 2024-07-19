package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

public enum Swatch {
    RED(0),
    ORANGE(15),
    YELLOW(23),
    GREEN(60),
    CYAN(90),
    BLUE(120),
    PURPLE(135),
    MAGENTA(150),
    BLACK(-1),
    WHITE(-2);

    private final int hue;

    Swatch(int hue) {
        this.hue = hue;
    }

    public int getHue() {
        return hue;
    }

    private static Map map = new HashMap<>();

    static {
        for (Swatch swatch : Swatch.values()) {
            map.put(swatch.hue, swatch);
        }
    }

    public static Swatch valueOf(int swatch) {
        return (Swatch) map.get(swatch);
    }
}
