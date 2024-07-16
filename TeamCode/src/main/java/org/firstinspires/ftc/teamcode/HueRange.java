package org.firstinspires.ftc.teamcode;

public class HueRange {
    private int hueCenter;    // 0 - 180
    private int hueSpan;     // 0 - 180

    private int hueMin = 0;    // 0 - 180
    private int hueMax = 180;   // 0 - 180
    private boolean invertRange = false;

    public HueRange () {
        setRange(90, 180);
    }

    public HueRange (int center, int span) {
        setRange(center, span);
    }

    public void setRange(int center, int span) {
        hueCenter = clamp(center, 0, 180);
        hueSpan = clamp(span, 0, 180);
        int half = hueSpan / 2;
        hueMin = hueCenter - half;
        hueMax = hueCenter + half;

        if (hueMin < 0) {
            hueMin += 180;
            invertRange = true;
        } else if (hueMax > 180) {
            hueMax -= 180;
            invertRange = true;
        } else {
            invertRange = false;
        }
    }

    public int center() { return hueCenter;}
    public int span() { return hueSpan;}
    public int min() { return hueMin;}
    public int max() { return hueMax;}
    public boolean invert() { return invertRange;}

    public String toString() {
        return String.format("Center Hue: %d, Span: %d", hueCenter, hueSpan);
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
