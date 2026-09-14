/* Copyright (c) 2026 Phil Malone
 * Another Mr. Phil Tutorial
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and turn towards an AprilTag Cluster.
 * A "Cluster" is a group of Apriltags that share a common origin, and are identified by name.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * Motors must be named: front_left_drive and front_right_drive, back_left_drive and back_right_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the BIOBUZZ AprilTag Library is being loaded by default
 *
 * Under manual control:
 *  The left stick will move forward/back & left/right.
 *  The right stick will rotate the robot.
 *    Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Point to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Point To Target" mode, the robot has one goal:
 *  Turn the robot to always keep the Target centered on the camera frame. (Use the Target Bearing to turn the robot.)
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name="Cluster Tracking", group = "A Tutorial")
public class ClusterTracking extends LinearOpMode
{
    //  Set the GAIN constant to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double TURN_GAIN      = 0.03;      //  Turn Control "Gain".  Multiply by heading error to get turn power.
    final double MAX_AUTO_TURN  = 0.4;       //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontLeftDrive = null;   //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor backLeftDrive = null;    //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;   //  Used to control the right back drive wheel

    private final boolean USE_WEBCAM = true; // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;       // Used to manage the video source.
    private AprilTagProcessor aprilTag;      // Used for managing the AprilTag detection process.
    private boolean areWeRed = false;        // Save which alliance color you are tracking
    private AprilTagClusterDetection targetCluster; // holds the detected cluster.

    private boolean targetFound  = false;    // Set to true when an AprilTag/Cluster target is detected
    private String targetName    = "none";
    private double targetRange   = 0;
    private double targetBearing = 0;
    private double targetYaw     = 0;

    @Override public void runOpMode()
    {
        // Initialize the Apriltag Detection process & drivetrain
        initAprilTag();
        initDrivetrain();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        while (opModeInInit()) {
            if (gamepad1.a) {
                areWeRed = false;
            } else if (gamepad1.b){
                areWeRed = true;
            }
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData("Alliance color", areWeRed ? "RED" : "BLUE");
            telemetry.addData(">", "cross-A/circle-B to set color.");
            telemetry.addData(">", "START to drive.");
            telemetry.update();
        };

        // Now drive the robot and track Apriltag
        while (opModeIsActive())
        {
            targetCluster = null;

            // establish default manual driving.  Slow things down to make the robot more controlable.
            double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
            double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
            double turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {

                if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                    // isolate the correct colored cluster that is on your side of the field.
                    if (clusterDet.metadata.name.contains(areWeRed ? "RED" : "BLUE") &&
                        Math.abs(clusterDet.ftcPose.roll) < 90) {
                        // Yes, we want to use this tag.
                        targetCluster = clusterDet;
                        targetName    = clusterDet.metadata.shortName;
                        targetRange   = clusterDet.ftcPose.range;
                        targetBearing = clusterDet.ftcPose.bearing;
                        targetYaw     = clusterDet.ftcPose.yaw;
                        break;  // don't look any further.
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetCluster != null) {
                telemetry.addData("\n>","HOLD Left-Bumper to Point to Target\n");
                telemetry.addData("Found", "%s", targetCluster.metadata.shortName);

                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", targetCluster.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", targetCluster.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", targetCluster.ftcPose.x, targetCluster.ftcPose.y, targetCluster.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", targetCluster.ftcPose.pitch, targetCluster.ftcPose.roll, targetCluster.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", targetCluster.ftcPose.range, targetCluster.ftcPose.bearing, targetCluster.ftcPose.elevation));

                // Add "key" information to telemetry
                telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                telemetry.addLine("RBE = Range, Bearing & Elevation");
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }
            telemetry.update();

            // If Left Bumper is being pressed, AND we have found the desired target, Point directly at it.
            if (gamepad1.left_bumper && targetFound) {

                // We want targetBearing to be 0, so use it as the "error" term to set the turn power.  Clip the result
                turn   = Range.clip(targetBearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

                telemetry.addData("Auto  ","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Initialize the Drivetrain
     */
    private void initDrivetrain() {
        // Initialize the drive motors.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(800, 600))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
