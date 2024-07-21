package org.firstinspires.ftc.teamcode;

/*
 *  This class is used to hold the portion of the Hue rainbow we are interested in finding.
 *  Note: This code assumes that the legal hue values reach from 0 to 180 (rather the 0-360 or 0-255 range)
 *
 *   When creating a HueRange, the range is specified as a "center" hue, and then a "span" of hues around this center.
 *   eg: A center hue of 15 (orange) with a span of 10, would then cover the hues starting at the redish hue of 10 and reaching to the yellowish hue of 20
 *
 *   If a span causes the range of hue values to wrap around the 0-180 boundary, the two ends are switched for a single
 *   region between the two segments, and then the color detection inverts the hue range to exclude the central hue region.
 *
 *   eg: A central hue of 5, with a span is 20, causes the desired range to be split into two regions...  0-15 and 175-180
 *   To search these two ranges, the central portion of 15-175 is used, with an Invert option.
 */
public class ColorRange {
    private int hueCenter;      // 0 - 179
    private int hueSpan;        // 0 - 179

    private int satMin;        // 0 - 255
    private int satMax;        // 0 - 255
    private int valMin;        // 0 - 255
    private int valMax;        // 0 - 255

    private int hueMin = 0;     // 0 - 179
    private int hueMax = 179;   // 0 - 179
    private boolean splitHue = false;

    public ColorRange() {
        setHueRange(90, 180);
        setSatRange(0, 255);
        setValRange(0, 255);
    }

    public ColorRange(int center, int span, int minSat, int satMax, int minVal, int maxVal) {
        setHueRange(center, span);
        setSatRange(minSat, satMax);
        setValRange(minVal, maxVal);
    }

    public void setHueRange(int center, int span) {
        hueCenter   = clamp(center, 0, 179);
        hueSpan     = clamp(span, 0, 179);
        hueMin      = hueCenter - (span / 2);
        hueMax      = hueCenter + (span / 2);

        // Allow search range to wrap around, and then invert the inner selected region.
        if (hueMin < 0) {
            int temp = hueMin + 180;
            hueMin   = hueMax;
            hueMax   = temp;
            splitHue = true;
        } else if (hueMax >= 180) {
            int temp = hueMax - 180;
            hueMax   = hueMin;
            hueMin   = temp;
            splitHue = true;
        } else {
            splitHue = false;
        }
    }

    public void setSatRange(int min, int max) {
        satMin = clamp(min, 0, 255);
        satMax = clamp(max, 0, 255);
    }

    public void setValRange(int min, int max) {
        valMin = clamp(min, 0, 255);
        valMax = clamp(max, 0, 255);
    }

    public int center() { return hueCenter;}
    public int span() { return hueSpan;}
    public int minH() { return hueMin;}
    public int maxH() { return hueMax;}
    public int minS() { return satMin;}
    public int maxS() { return satMax;}
    public int minV() { return valMin;}
    public int maxV() { return valMax;}
    public boolean split() { return splitHue;}

    public String toString() {
        return String.format("C: %d, S: %d (%d, %d, %s) S[%d, %d] V[%d, %d]", hueCenter, hueSpan,
                                    hueMin, hueMax, splitHue, satMin, satMax, valMin, valMax);
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
