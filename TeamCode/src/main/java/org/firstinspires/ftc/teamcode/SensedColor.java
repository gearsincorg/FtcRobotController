package org.firstinspires.ftc.teamcode;

public class SensedColor {
    private Swatch  matchingSwatch;
    private int     detectedHue;
    private int     detectedSat;
    private int     detectedVal;

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
        detectedSat     = sat;
        detectedVal     = val;
    }

    // getters
    public Swatch swatch() {return matchingSwatch;}
    public int hue() {return detectedHue;}
    public int sat() {return detectedSat;}
    public int val() {return detectedVal;}

    // To display contents as human readable.
    public String toString() {
        return String.format("%s (H:%3d S:%3d V:%3d)", matchingSwatch, detectedHue, detectedSat, detectedVal);
    }
}


