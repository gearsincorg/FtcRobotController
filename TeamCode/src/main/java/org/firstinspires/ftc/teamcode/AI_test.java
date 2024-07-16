package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "TurnToGreenOpMode", group = "Autonomous")
public class AI_test extends LinearOpMode {

    // Declare robot hardware variables
    private DcMotor motorLeft;
    private DcMotor motorRight;

    // Declare vision variables
    private OpenCvCamera camera;
    private GreenDetectionPipeline pipeline;

    // Camera configuration
    private static final int CAMERA_WIDTH  = 640; // width  of video frame captured from camera
    private static final int CAMERA_HEIGHT = 480; // height of video frame captured from camera

    // Tuning parameters for turning
    private static final double TURN_POWER = 0.2; // Power for turning motors
    private static final double ERROR_THRESHOLD = 10; // Threshold for acceptable error in center

    @Override
    public void runOpMode() {
        // Initialize hardware
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");

        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Create green detection pipeline
        pipeline = new GreenDetectionPipeline();
        camera.setPipeline(pipeline);

        // Start camera streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        // Wait for start signal
        waitForStart();

        // Turn towards green object
        while (opModeIsActive() && !pipeline.greenDetected()) {
            // Wait until green is detected
            telemetry.addData("Green", "Not Detected!");
            telemetry.update();
            sleep(100); // Small delay to prevent excessive polling
        }

        // Turn until green object is centered in the camera
//        while (opModeIsActive() && Math.abs(pipeline.getXCenter() - CAMERA_WIDTH / 2) > ERROR_THRESHOLD) {
          while (opModeIsActive()) {
            // Calculate error
            double error = pipeline.getXCenter() - CAMERA_WIDTH / 2;

            // Turn left if green object is on the left
            if (error < 0) {
                motorLeft.setPower(TURN_POWER);
                motorRight.setPower(-TURN_POWER);
            } else {
                // Turn right if green object is on the right
                motorLeft.setPower(-TURN_POWER);
                motorRight.setPower(TURN_POWER);
            }

            // Update telemetry
            telemetry.addData("X Center", pipeline.getXCenter());
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // Stop motors
        motorLeft.setPower(0);
        motorRight.setPower(0);

        // Green object is centered
        telemetry.addData("Green", "Centered!");
        telemetry.update();
    }

    // Custom pipeline for green detection
    static class GreenDetectionPipeline extends OpenCvPipeline {

        private static final Scalar GREEN_LOWER = new Scalar(50, 50, 50);
        private static final Scalar GREEN_UPPER = new Scalar(100, 255, 255);
        private static final Scalar BLUE = new Scalar(7, 192, 255);

        private Mat mask = new Mat();
        private boolean greenDetected = false;
        private double xCenter = 0;

        @Override
        public Mat processFrame(Mat input) {
            // Convert input to HSV color space
            Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);

            // Threshold HSV image to get a mask for green color
            Core.inRange(input, GREEN_LOWER, GREEN_UPPER, mask);

            // Find contours in the mask
            java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Check if green color is detected
            greenDetected = contours.size() > 0;

            // Calculate center of green object (if detected)
            if (greenDetected) {
                // Calculate bounding rectangle for the contour
                Rect rect = Imgproc.boundingRect(contours.get(0));
                xCenter = rect.x + rect.width / 2;

                // Draw bounding rectangle on the output image
                Imgproc.rectangle(input, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), BLUE, 2);
            } else {
                xCenter = 0;
            }

            return input;
        }

        public boolean greenDetected() {
            return greenDetected;
        }

        public double getXCenter() {
            return xCenter;
        }
    }
}