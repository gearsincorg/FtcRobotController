/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * This OpMode illustrates using the ColorSensorProcessor to determine the color of a portion of the camera view.
 *
 */
@TeleOp(name="Video Color Sensor", group = "Sensor")
public class ConceptColorSensor extends LinearOpMode
{

    // Create a VisionPortal and ColorSensorProcessor
    private VisionPortal         myVisionPortal;
    private ColorSensorProcessor colorSensorProcessor;
    private ColorWOI colorWIO;

    @Override public void runOpMode()
    {
        initVisionPortal();

        // Run this code during the Init period so the Driver Station can display the video window.
        while(opModeInInit()) {
            // Display the detected color
            telemetry.addLine(colorSensorProcessor.getSensedColor().toString());
            telemetry.addLine(colorWIO.getOpenCVRect(320,240).toString());

            telemetry.update();
        }
    }

    /**
     * Initialize the visionPortal and ColorSensor
     */
    private void initVisionPortal() {
        // -----------------------------------------------------------------------------------------
        // Create a color sensor processor and pass it the desired window of interest
        //  (smaller window provides faster processing frame rate)
        // -----------------------------------------------------------------------------------------
        colorWIO = new ColorWOI(ColorWOI.DefineType.UNITY_CENTER_ORIGIN, 0, 0, 0.25, 0.25);
        colorSensorProcessor = new ColorSensorProcessor(colorWIO);

        // -----------------------------------------------------------------------------------------
        // Camera Configuration (lower resolution provides faster processing frame rate)
        // -----------------------------------------------------------------------------------------
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(colorSensorProcessor)
                .setCameraResolution(new Size(320, 240))
                .build();
    }
}
