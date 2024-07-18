package org.firstinspires.ftc.teamcode;

import org.opencv.core.Rect;

public class ColorWOI {

    public enum DefineType {
        IMAGE_PIXELS,
        GRAPH_STYLE_PIXELS,
        UNITY_BOTTOM_LEFT_ORIGIN,
        UNITY_CENTER_ORIGIN
    }

    private DefineType  defineType;
    private double  x;
    private double  y;
    private double  width;
    private double  height;

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

    public Rect getOpenCVRect(int srcWidth, int srcHeight) {
        Rect openCVRect;

        switch (defineType) {
            case IMAGE_PIXELS: {
                openCVRect = new Rect((int)x, (int)y, (int)width, (int)height);
                break;
            }

            case GRAPH_STYLE_PIXELS: {
                openCVRect = new Rect((int)x,
                                      (int)(srcHeight - (y + height)),
                                      (int)width,
                                      (int)height);
                break;
            }

            case UNITY_BOTTOM_LEFT_ORIGIN: {
                openCVRect = new Rect((int)(srcWidth * x),
                                      (int)(srcHeight - (srcHeight * (y + height))),
                                      (int)(srcWidth  * width),
                                      (int)(srcHeight * height));
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

