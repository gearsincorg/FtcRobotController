/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * This OpMode illustrates using the ColorSensorProcessor to determine the color of a portion of the camera view.
 *
 */
@TeleOp(name="Video Color Finder", group = "Sensor")
public class ConceptColorFinder extends LinearOpMode
{
    // Define a window of interest (WOI) to be processed by the Color Finder.
    // The WOI can be defined using two styles:
    //
    // OPENCV_STANDARD.     OpenCV standard Rect:    Origin at upper left, measurement units: pixels.
    //                                               +X is to the right, +ve Y is down
    // eg: new ColorWOI(ColorWOI.DefineType.OPENCV_LOPLEFT_ORIGIN, 60, 45, 40, 30);  // same as example below

    // UNITY_CENTER_ORIGIN. Normalized coordinates:  Origins at image/rect center. Source image size is +/- 1 in both X and Y dimensions.
    //                                               Size of 1 unit is half width (or height) of source image
    //                                               +X is to the right, +ve Y is up
    // eg: new ColorWOI(ColorWOI.DefineType.UNITY_CENTER_ORIGIN, -0.5, 0.5, 0.25, 0.25);  // same as example above

    private ColorWOI colorWIO = new ColorWOI(ColorWOI.DefineType.UNITY_CENTER_ORIGIN, 0.0, 0.0, 0.8, 0.8);

    // Define the range of colors you are looking for by choosing a center Hue and a Span (range) of hues around that center.
    // Here, Hues range from 0 to 180 with Red wrapping around both ends of the range
    // Example hueRanges:
    // RED      HueRange(170, 40);
    // YELLOW   HueRange( 27, 20);
    // GREEN    HueRange( 60, 40);
    // BLUE     HueRange(120, 40);

    private ColorRange colorRange = new ColorRange(120, 40, 100, 255, 110, 255);

    // Create a VisionPortal and ColorSensorProcessor
    private VisionPortal         myVisionPortal;
    private ColorFinderProcessor colorFinderProcessor;

    private boolean lastBumpUp = false;
    private boolean lastBumpDn = false;

    @Override public void runOpMode()
    {
        initVisionPortal();

        // Run this code during the Init period so the Driver Station can display the video window.
        while(opModeInInit()) {

            bumpColor();
            // Display the detected color
            telemetry.addLine("use \'Menu - Camera Stream\' to view color Finder\n");
            telemetry.addData("Rectangle ", colorWIO.getOpenCVRect(320,240).toString());
            telemetry.addLine(colorRange.toString());
            telemetry.addLine(colorFinderProcessor.getFoundBlobs().toString());
            telemetry.update();
        }
    }

    /**
     * Initialize the visionPortal and ColorSensor
     */
    private void initVisionPortal() {
        // -----------------------------------------------------------------------------------------
        // Create a color sensor processor and pass it the desired window of interest
        //  (smaller window provides faster processing frame rate), and a list of the
        //  color swatches you are interested in.
        // -----------------------------------------------------------------------------------------
        colorFinderProcessor = new ColorFinderProcessor(colorWIO, colorRange);

        // -----------------------------------------------------------------------------------------
        // Camera Configuration (lower resolution provides faster processing frame rate)
        // -----------------------------------------------------------------------------------------
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(colorFinderProcessor)
                .setCameraResolution(new Size(320, 240))
                .build();
    }

    private void bumpColor() {
        boolean thisBumpUp = gamepad1.dpad_up;
        boolean thisBumpDn = gamepad1.dpad_down;

        if (thisBumpUp && !lastBumpUp) {
            if (gamepad1.left_bumper) {
                colorRange.setHueRange(colorRange.center() + 1, colorRange.span());
            } else if (gamepad1.right_bumper) {
                colorRange.setHueRange(colorRange.center(), colorRange.span() + 1);
            } else if (gamepad1.square) {
                colorRange.setSatRange(colorRange.minS() + 1, colorRange.maxS());
            } else if (gamepad1.circle) {
                colorRange.setSatRange(colorRange.minS(), colorRange.maxS() + 1);
            } else if (gamepad1.cross) {
                colorRange.setValRange(colorRange.minV() + 1, colorRange.maxV());
            } else if (gamepad1.triangle) {
                colorRange.setValRange(colorRange.minV(), colorRange.maxV() + 1);
            }

            colorFinderProcessor.setColorRange(colorRange);

        } else if (thisBumpDn && !lastBumpDn) {
            if (gamepad1.left_bumper) {
                colorRange.setHueRange(colorRange.center() - 1, colorRange.span());
            } else if (gamepad1.right_bumper) {
                colorRange.setHueRange(colorRange.center(), colorRange.span() - 1);
            } else if (gamepad1.square) {
                colorRange.setSatRange(colorRange.minS() - 1, colorRange.maxS());
            } else if (gamepad1.circle) {
                colorRange.setSatRange(colorRange.minS(), colorRange.maxS() - 1);
            } else if (gamepad1.cross) {
                colorRange.setValRange(colorRange.minV() - 1, colorRange.maxV());
            } else if (gamepad1.triangle) {
                colorRange.setValRange(colorRange.minV(), colorRange.maxV() - 1);
            }

            colorFinderProcessor.setColorRange(colorRange);
        }

        lastBumpUp = thisBumpUp;
        lastBumpDn = thisBumpDn;
    }
}
