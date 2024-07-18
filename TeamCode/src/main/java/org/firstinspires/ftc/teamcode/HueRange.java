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
public class HueRange {
    private int hueCenter;      // 0 - 180
    private int hueSpan;        // 0 - 180

    private int hueMin = 0;     // 0 - 180
    private int hueMax = 180;   // 0 - 180
    private boolean invertRange = false;

    public HueRange () {
        setRange(90, 180);
    }

    public HueRange (int center, int span) {
        setRange(center, span);
    }

    public void setRange(int center, int span) {
        int half    = hueSpan / 2;
        hueCenter   = clamp(center, 0, 180);
        hueSpan     = clamp(span, 0, 180);
        hueMin      = hueCenter - half;
        hueMax      = hueCenter + half;

        // Allow search range to wrap around, and then invert the inner selected region.
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
        return String.format("Center: %d, Span: %d", hueCenter, hueSpan);
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
