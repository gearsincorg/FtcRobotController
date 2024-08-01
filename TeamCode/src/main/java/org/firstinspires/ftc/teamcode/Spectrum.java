package org.firstinspires.ftc.teamcode;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Spectrum {

    /***
     * Convert single HSV pixel value to a YCrCb pixel
     * @param  hsv   value
     * @return YCrCb value
     */
    public static Scalar hsv2yCrCb(Scalar hsv) {

        // Convert to yCrCv
        Mat pixel = new Mat(1, 1, CvType.CV_8UC3, hsv);
        Imgproc.cvtColor(pixel, pixel, Imgproc.COLOR_HSV2BGR);
        Imgproc.cvtColor(pixel, pixel, Imgproc.COLOR_BGR2YCrCb);

        // Return YCrCb values
        return new Scalar(pixel.get(0, 0));
    }

    /***
     * Convert a single YCrCb pixel value to a HSV pixel
     * @param yCrCv value
     * @return hsv value
     */
    public static Scalar yCrCb2hsv(Scalar yCrCv) {

        // Convert to HSV
        Mat pixel = new Mat(1, 1, CvType.CV_8UC3, yCrCv);
        Imgproc.cvtColor(pixel, pixel, Imgproc.COLOR_YCrCb2BGR);
        Imgproc.cvtColor(pixel, pixel, Imgproc.COLOR_BGR2HSV);

        // Return HSV values
        return new Scalar(pixel.get(0, 0));
    }
}
