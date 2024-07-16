/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Color Guessor", group = "AAA")
public class HueGuessor extends LinearOpMode
{

    // get an instance of the "Robot" class, then instanciate all other subsystems
    private HuePipeline hueProcessor;
    private VisionPortal myVisionPortal;

    @Override public void runOpMode()
    {
        initVisionPortal();

        while(opModeInInit()) {

            telemetry.addLine(hueProcessor.getDetectedColor().toString());
            telemetry.update();
        }
    }

    /**
     * Initialize AprilTag and TFOD.
     */
    private void initVisionPortal() {
        // -----------------------------------------------------------------------------------------
        // Team Prop Configuration
        // -----------------------------------------------------------------------------------------
        hueProcessor = new HuePipeline(160, 120, 50, 25);

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(hueProcessor)
                .setCameraResolution(new Size(320, 240))
                .build();
    }
}
