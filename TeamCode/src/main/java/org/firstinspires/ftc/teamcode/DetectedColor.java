package org.firstinspires.ftc.teamcode;

public class DetectedColor {
    private ColorSwatch detectedSwatch;
    private int[]       detectedHSV;

    public DetectedColor(ColorSwatch swatch) {
        detectedSwatch = swatch;
        detectedHSV = null;
    }
    public DetectedColor(ColorSwatch swatch, int hue) {
        detectedSwatch = swatch;
        detectedHSV = new int[] {hue, 255, 255};
    }

    public DetectedColor(ColorSwatch swatch, int hue, int sat, int val) {
        detectedSwatch = swatch;
        detectedHSV = new int[] {hue, sat, val};
    }

    public ColorSwatch swatch() {return detectedSwatch;}
    public int[] hsv() {return detectedHSV;}

    public String toString() {
        return String.format("%s  [%d, %d, %d]", detectedSwatch, detectedHSV[0], detectedHSV[1], detectedHSV[2] );
    }
}


