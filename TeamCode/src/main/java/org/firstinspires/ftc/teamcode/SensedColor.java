package org.firstinspires.ftc.teamcode;

public class SensedColor {
    private int         detectedHue;
    private Swatch      matchingSwatch;
    private int         avgSaturation;
    private int         avgValue;

    public SensedColor() {
        setContent(Swatch.BLACK, 0, 0, 0);
    }

    public SensedColor(Swatch swatch) {
        setContent(swatch, 0, 0, 0);
    }

    public SensedColor(Swatch swatch, int hue, int sat, int val) {
        setContent(swatch, hue, sat, val);
    }

    private void setContent(Swatch swatch, int hue, int sat, int val){
        matchingSwatch  = swatch;
        detectedHue     = hue;
        avgSaturation   = sat;
        avgValue        = val;
    }

    // getters
    public Swatch swatch() {return matchingSwatch;}
    public int          hue() {return detectedHue;}

    // To display contents as human readable.
    public String toString() {
        return String.format("%s (H:%3d S:%3d V:%3d)", matchingSwatch, detectedHue, avgSaturation, avgValue);
    }
}


