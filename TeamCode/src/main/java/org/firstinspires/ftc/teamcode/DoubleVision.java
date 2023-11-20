/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;

/*
 * This OpMode illustrates the basics of using both AprilTag recognition and TensorFlow
 * Object Detection.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Double Vision", group = "Concept")
@Disabled
public class DoubleVision extends LinearOpMode {
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private ColourMassDetectionProcessor EOCVColorDetection;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;

    @Override
    public void runOpMode() {
        initDoubleVision();

        // This OpMode loops continuously, allowing the user to switch between
        // AprilTag and TensorFlow Object Detection (TFOD) image processors.
        while (!isStopRequested())  {

            if (opModeInInit()) {
                telemetry.addData("DS preview on/off","3 dots, Camera Stream");
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
            }

            if (myVisionPortal.getProcessorEnabled(aprilTag)) {
                // User instructions: Dpad left or Dpad right.
                telemetry.addLine("Dpad Left to disable AprilTag");
                telemetry.addLine();
                telemetryAprilTag();
            } else {
                telemetry.addLine("Dpad Right to enable AprilTag");
            }
            telemetry.addLine();
            telemetry.addLine("----------------------------------------");
            if (myVisionPortal.getProcessorEnabled(EOCVColorDetection)) {
                telemetry.addLine("Dpad Down to disable EOCV");
                telemetry.addLine();
                telemetryEOCV();
            } else {
                telemetry.addLine("Dpad Up to enable EOCV");
            }

            // Push telemetry to the Driver Station.
            telemetry.update();

            if (gamepad1.dpad_left) {
                myVisionPortal.setProcessorEnabled(aprilTag, false);
            } else if (gamepad1.dpad_right) {
                myVisionPortal.setProcessorEnabled(aprilTag, true);
            }
            if (gamepad1.dpad_down) {
                myVisionPortal.setProcessorEnabled(EOCVColorDetection, false);
            } else if (gamepad1.dpad_up) {
                myVisionPortal.setProcessorEnabled(EOCVColorDetection, true);
            }
            sleep(20);

        }   // end while loop
    }   // end method runOpMode()


    /**
     * Initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
            .build();

        // -----------------------------------------------------------------------------------------
        // EOCV Configuration
        // -----------------------------------------------------------------------------------------

        // the current range set by lower and upper is the full range
        // HSV takes the form: (HUE, SATURATION, VALUE)
        // which means to select our colour, only need to change HUE
        // the domains are: ([0, 180], [0, 255], [0, 255])
        // this is tuned to detect red, so you will need to experiment to fine tune it for your robot
        // and experiment to fine tune it for blue
        Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
        double minArea = 100; // the minimum area for the detection to consider for your prop
        double leftLine = 213 ;
        double rightLine = 426 ;

        // Scalar lower = new Scalar(90, 160, 90); // the lower hsv threshold for your detection
        // Scalar upper = new Scalar(105, 200, 255); // the upper hsv threshold for your detection
        EOCVColorDetection = new ColourMassDetectionProcessor(lower, upper, minArea, leftLine, rightLine);

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        myVisionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessors(EOCVColorDetection, aprilTag)
            .build();
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryEOCV() {
        telemetry.addData("Currently Recorded Position", EOCVColorDetection.getRecordedPropPosition());
        telemetry.addData("Camera State", myVisionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + EOCVColorDetection.getLargestContourX() + ", y: " + EOCVColorDetection.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", EOCVColorDetection.getLargestContourArea());
    }   // end method telemetryTfod()

}   // end class
