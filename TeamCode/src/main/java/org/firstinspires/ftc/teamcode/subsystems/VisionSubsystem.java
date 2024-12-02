package org.firstinspires.ftc.teamcode.subsystems;

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

    private LinearOpMode myOpMode;
    private boolean showTelemetry     = false;

    private final int CAMERA_WIDTH = 320;
    private final int CAMERA_HEIGHT = 240;
    private final int MIDDLE_WIDTH = CAMERA_WIDTH/2;
    private final int MIDDLE_HEIGHT = CAMERA_HEIGHT/2;

    ColorBlobLocatorProcessor colorLocatorRed;
    ColorBlobLocatorProcessor colorLocatorBlue;
    VisionPortal portal ;

    // Vision Constructor
    public VisionSubsystem(LinearOpMode opmode) {myOpMode = opmode;}

    public void initilaize(boolean showTelemetry){

        // Create Color Blob Processor for vision system
        colorLocatorRed = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.5, 1, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                      // Don't contours on the Stream Preview
                .setBlurSize(5)                              // Smooth the transitions between different colors in image
                .build();

        colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.5, 1, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                      // Don't contours on the Stream Preview
                .setBlurSize(5)                              // Smooth the transitions between different colors in image
                .build();

        // Attach to camera and add BlobLocator
        portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorRed)
                .addProcessor(colorLocatorBlue)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    public ColorTarget getTarget (){

        ColorTarget target = new ColorTarget();
        double   centerX = 0;
        double   centerY = 0;
        String   colorMatch ;

        List<ColorBlobLocatorProcessor.Blob> blobs;
        if (Globals.ALLIANCE_COLOR == AllianceColor.RED) {
            blobs = colorLocatorRed.getBlobs();
        } else {
            blobs = colorLocatorBlue.getBlobs();
        }

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.
        if (!blobs.isEmpty()){
            ColorBlobLocatorProcessor.Blob bigBlob = blobs.get(0);
            RotatedRect boxFit = bigBlob.getBoxFit();
            centerX = (boxFit.center.x - MIDDLE_WIDTH) / MIDDLE_WIDTH;
            centerY = (boxFit.center.y - MIDDLE_HEIGHT) / MIDDLE_HEIGHT;

            target = new ColorTarget(centerX, centerY);

            if (showTelemetry){
                myOpMode.telemetry.addLine(String.format("%s: A:%5d, X %4.2f, Y %4.2f",
                        Globals.ALLIANCE_COLOR, bigBlob.getContourArea(), centerX, centerY));
            }
        } else {
            if (showTelemetry){
                myOpMode.telemetry.addLine("no targets found");
            }
        }


        return target;
    }
}


