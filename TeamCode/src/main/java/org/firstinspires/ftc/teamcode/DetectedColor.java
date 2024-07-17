package org.firstinspires.ftc.teamcode;

public class DetectedColor {
    private int         detectedHue;
    private ColorSwatch matchingSwatch;
    private int         avgSaturation;
    private int         avgValue;

    public DetectedColor() {
        matchingSwatch = ColorSwatch.BLACK;
        detectedHue    = 0;
        avgSaturation  = 0;
        avgValue       = 0;
    }

    public DetectedColor(ColorSwatch swatch) {
        matchingSwatch  = swatch;
        detectedHue     = 0;
        avgSaturation   = 0;
        avgValue        = 0;
    }

    public DetectedColor(ColorSwatch swatch, int hue, int sat, int val) {
        matchingSwatch  = swatch;
        detectedHue     = hue;
        avgSaturation   = sat;
        avgValue        = val;
    }


    public ColorSwatch swatch() {return matchingSwatch;}
    public int hue() {return detectedHue;}

    public String toString() {
        return String.format("%8s (%3d %3d %3d)", matchingSwatch, detectedHue, avgSaturation, avgValue);
    }
}


