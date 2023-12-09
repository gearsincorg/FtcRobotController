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

public class Vision {
    public VisionPortalState portalState = VisionPortalState.NONE;
    public AprilTagProcessor aprilTag;

    private LinearOpMode myOpMode;
    private TeamPropPipeline teamProp;
    private VisionPortal myVisionPortal;

    private boolean showTelemetry;

    public Vision (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initialize(boolean showTelemetry) {
        initVisionPortal();
        myVisionPortal.setProcessorEnabled(aprilTag, false);
        myVisionPortal.setProcessorEnabled(teamProp, false);
        this.showTelemetry = showTelemetry;
    }

    public void enableAprilTag() {
        myVisionPortal.setProcessorEnabled(teamProp, false);
        setManualExposure(8, 255);
        myVisionPortal.setProcessorEnabled(aprilTag, true);
        portalState = VisionPortalState.APRILTAG;
    }

    public void enableTeamProp() {
        myVisionPortal.setProcessorEnabled(aprilTag, false);
        setAutoExposure();
        myVisionPortal.setProcessorEnabled(teamProp, true);
        portalState = VisionPortalState.TEAM_PROP;
    }

    public void disableAll() {
        myVisionPortal.setProcessorEnabled(teamProp, false);
        myVisionPortal.setProcessorEnabled(aprilTag, false);
        portalState = VisionPortalState.NONE;
    }

    public TeamPropLocation getTeamPropLocation() {
        return teamProp.getTeamPropLocation();
    }
    public int getContourCount() { return teamProp.getContourCount();}

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        myOpMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                myOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                myOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }   // end for() loop
    }

    public Target findStack() {
        Target stack = new Target();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((detection.id == 8 ) || (detection.id == 9 )) {
                    stack.targetFound = true;
                    stack.rangeInch = detection.ftcPose.range;
                    stack.bearingDeg = detection.ftcPose.bearing;
                    stack.x = detection.ftcPose.x;

                    if (showTelemetry) {
                        myOpMode.telemetry.addLine(String.format("Stack %d  R:B %6.1f %6.1f", detection.id, detection.ftcPose.range, detection.ftcPose.bearing));
                    }
                    break;
                }
            }
        }
        return stack;
    }

    /**
     * Add telemetry about TeamProp detection.
     */
    public void telemetryTeamProp() {
        myOpMode.telemetry.addData("Location", teamProp.getTeamPropLocation());
        myOpMode.telemetry.addData("Camera State", myVisionPortal.getCameraState());
        myOpMode.telemetry.addLine(teamProp.getTargetString());
    }

    /**
     * Initialize AprilTag and TFOD.
     */
    private void initVisionPortal() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        aprilTag.setDecimation(2);

        // -----------------------------------------------------------------------------------------
        // Team Prop Configuration
        // -----------------------------------------------------------------------------------------
        teamProp = new TeamPropPipeline();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(teamProp, aprilTag)
                .setCameraResolution(new Size(640, 360))
                .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (myVisionPortal == null) {
            return;
        }

        waitForStream();

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested())
        {
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = myVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpMode.sleep(20);
        }
    }

    private void    setAutoExposure() {
        // Wait for the camera to be open, then use the controls
        if (myVisionPortal == null) {
            return;
        }

        waitForStream();

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested())
        {
            ExposureControl exposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.AperturePriority) {
                exposureControl.setMode(ExposureControl.Mode.AperturePriority);
                myOpMode.sleep(50);
            }
        }
    }


    public void waitForStream() {
        // Make sure camera is streaming before we try to set the exposure controls
        if (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                myOpMode.sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }
    }
}
