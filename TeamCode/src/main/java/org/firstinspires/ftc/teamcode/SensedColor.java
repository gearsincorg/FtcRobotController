package org.firstinspires.ftc.teamcode;

public class SensedColor {
    private int         detectedHue;
    private ColorSwatch matchingSwatch;
    private int         avgSaturation;
    private int         avgValue;

    public SensedColor() {
        setContent(ColorSwatch.BLACK, 0, 0, 0);
    }

    public SensedColor(ColorSwatch swatch) {
        setContent(swatch, 0, 0, 0);
    }

    public SensedColor(ColorSwatch swatch, int hue, int sat, int val) {
        setContent(swatch, hue, sat, val);
    }

    private void setContent(ColorSwatch swatch, int hue, int sat, int val){
        matchingSwatch  = swatch;
        detectedHue     = hue;
        avgSaturation   = sat;
        avgValue        = val;
    }

    // getters
    public ColorSwatch  swatch() {return matchingSwatch;}
    public int          hue() {return detectedHue;}

    // To display contents as human readable.
    public String toString() {
        return String.format("%8s (%3d %3d %3d)", matchingSwatch, detectedHue, avgSaturation, avgValue);
    }
}


