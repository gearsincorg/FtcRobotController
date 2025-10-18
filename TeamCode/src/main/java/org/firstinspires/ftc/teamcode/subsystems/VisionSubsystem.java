package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionSubsystem {

    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    LinearOpMode myOpmode;

    private int     myExposure  ;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain      ;
    private int     minGain ;
    private int     maxGain ;
    private boolean showTelemetry;
    private double  range   = 0;
    private double  bearing = 0;

    public VisionSubsystem(LinearOpMode opmode){
        myOpmode = opmode;
    }

    public void init (boolean showTelemetry){

        this.showTelemetry = showTelemetry;

        // Initialize the Apriltag Detection process
        initAprilTag();
        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getCameraSetting();
        myExposure = Math.min(5, minExposure);
        myGain = maxGain;
        setManualExposure(myExposure, myGain);
    }

    public void update() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        myOpmode.telemetry.addData("# AprilTags Detected", currentDetections.size());
        range   = 0;
        bearing = 0;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            range   = detection.ftcPose.range;
            bearing = detection.ftcPose.bearing;

            if (showTelemetry) {

                if (detection.metadata != null) {
                    myOpmode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    myOpmode.telemetry.addLine(String.format("Range %6.1f in, Bearing %6.1f deg.", range, bearing));
                }
            }
        }   // end for() loop
    }   // end method telemetryAprilTag()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        aprilTag.setDecimation(3);

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpmode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480 ))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();
    }

    private void waitForCamera() {
        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpmode.telemetry.addData("Camera", "Waiting");
            myOpmode.telemetry.update();
            while (!myOpmode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                myOpmode.sleep(20);
            }
            myOpmode.telemetry.addData("Camera", "Ready");
            myOpmode.telemetry.update();
        }
    }

    /*
    Manually set the camera gain and exposure.
    Can only be called AFTER calling initAprilTag();
    Returns true if controls are set.
 */
    private boolean    setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        waitForCamera();

        // Set camera controls unless we are stopping.
        if (!myOpmode.isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpmode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            myOpmode.sleep(20);
            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpmode.sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        waitForCamera();

        // Get camera control values unless we are stopping.
        if (!myOpmode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }

    public double getBearing(){
        return bearing;
    }

    public double getRange() {
        return bearing;
    }
}
