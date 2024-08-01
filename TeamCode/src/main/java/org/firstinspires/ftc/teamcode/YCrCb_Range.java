package org.firstinspires.ftc.teamcode;

import org.opencv.core.Scalar;

public class YCrCb_Range {
    public Scalar  loRange;
    public Scalar  hiRange;

    public YCrCb_Range (Scalar loRange, Scalar hiRange) {
        this.loRange = loRange;
        this.hiRange = hiRange;
    }

    public double lo_Y ()  { return loRange.val[0];}
    public double hi_Y ()  { return hiRange.val[0];}
    public double lo_Cr () { return loRange.val[1];}
    public double hi_Cr () { return hiRange.val[1];}
    public double lo_Cb () { return loRange.val[2];}
    public double hi_Cb () { return hiRange.val[2];}

    public String toString () {
        return String.format("YCrCb Lo %s, Hi %s", loRange.toString(), hiRange.toString());
    }
}

