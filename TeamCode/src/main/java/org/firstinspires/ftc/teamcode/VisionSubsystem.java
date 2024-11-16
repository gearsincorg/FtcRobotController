package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

public class VisionSubsystem {

    private final int CAMERA_WIDTH = 320;
    private final int CAMERA_HEIGHT = 240;
    private final int MIDDLE_VALUE = CAMERA_WIDTH/2;

    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;
    ColorBlobLocatorProcessor colorLocator;

    // Vision Constructor
    public VisionSubsystem(LinearOpMode opmode) {myOpMode = opmode;}

    public void initilaize(boolean showTelemetry){

       colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public double getTargetX (){

        double   center = 0;

        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.
        if (!blobs.isEmpty()){
            ColorBlobLocatorProcessor.Blob bigBlob = blobs.get(0);
            RotatedRect boxFit = bigBlob.getBoxFit();
            center = boxFit.center.x;
            center = (center - MIDDLE_VALUE) / MIDDLE_VALUE;

            if (showTelemetry){
                myOpMode.telemetry.addLine(String.format(" target: %5d  %4.2f   %5.2f  (%3d,%3d)",
                        bigBlob.getContourArea(), bigBlob.getDensity(), center, (int) boxFit.center.x, (int) boxFit.center.y));
            }
        } else {
            if (showTelemetry){
                myOpMode.telemetry.addLine("no targets found");
            }
        }


        return center;
    }
}
