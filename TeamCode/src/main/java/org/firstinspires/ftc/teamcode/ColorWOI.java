package org.firstinspires.ftc.teamcode;

import org.opencv.core.Rect;

public class ColorWOI {

    public enum DefineType {
        OPENCV_TOPLEFT_ORIGIN,
        UNITY_CENTER_ORIGIN
    }

    private DefineType  defineType;
    private double  x;
    private double  y;
    private double  width;
    private double  height;
    private int  srcWidth;
    private int  srcHeight;

    public ColorWOI (){
        this.defineType = DefineType.UNITY_CENTER_ORIGIN;
        this.x = 0;
        this.y = 0;
        this.width = 2;
        this.height = 2;
    }

    public ColorWOI (DefineType defineType, double x, double y, double width, double height){
        this.defineType = defineType;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    public ColorWOI (DefineType defineType, int x, int y, int width, int height){
        this.defineType = defineType;
        this.x = (double)x;
        this.y = (double)y;
        this.width = (double)width;
        this.height = (double)height;
    }

    public void setSrc(int srcWidth, int srcHeight) {
        this.srcWidth = srcWidth;
        this.srcHeight = srcHeight;
    }

    public Rect getOpenCVRect(int srcWidth, int srcHeight) {
        Rect openCVRect;
        this.srcWidth = srcWidth;
        this.srcHeight = srcHeight;

        switch (defineType) {
            case OPENCV_TOPLEFT_ORIGIN: {
                openCVRect = new Rect((int)x, (int)y, (int)width, (int)height);
                break;
            }

            case UNITY_CENTER_ORIGIN: {
                double halfWidth  = srcWidth / 2.0;  // used to locate center AND as standard unit size
                double halfHeight = srcHeight / 2.0; // used to locate center AND as standard unit size
                openCVRect = new Rect((int)(halfWidth + (halfWidth * (x - (width / 2.0)))),
                                      (int)(halfHeight - (halfHeight * (y + (height / 2.0)))),
                                      (int)(halfWidth  * width),
                                      (int)(halfHeight * height));
                break;
            }

            default: {
                openCVRect = new Rect();
            }
        }

        // Adjust the window position to ensure it stays on the screen.  push it back into the screen area.
        // We could just crop it instead, but then it may completely miss the screen.
        openCVRect.x = Math.max(openCVRect.x, 0);
        openCVRect.x = Math.min(openCVRect.x, srcWidth - openCVRect.width);
        openCVRect.y = Math.max(openCVRect.y, 0);
        openCVRect.y = Math.min(openCVRect.y, srcHeight - openCVRect.height);

        return openCVRect;
    };

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}

