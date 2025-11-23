package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;
import org.firstinspires.ftc.teamcode.auxtools.Target;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionSubsystem extends SubsystemBase {

    private final int BLUE_GOAL_ID = 20;
    private final int RED_GOAL_ID  = 24;

    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    private int     myExposure  ;
    private int     minExposure ;
    private int     myGain      ;
    private int     maxGain ;

    private double  range   = 0;
    private double  bearing = 0;

    public VisionSubsystem(LinearOpMode myOpMode) {
        super(myOpMode);
    }

    @Override
    public void init (boolean showTelemetry){
        super.init(showTelemetry);

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getCameraSetting();
        myExposure =  Math.min(3, minExposure);
        myGain     =  20;
        setManualExposure(myExposure, myGain);
    }

    /**
     * Read any sensor for this subsystem and calculate any derived values
     * Called every Update() cycle;
     */
    public Target findTarget() {
        int targetTagID = (Globals.ALLIANCE_COLOR == AllianceColor.RED) ? RED_GOAL_ID : BLUE_GOAL_ID;
        Target target = new Target();

        if (subsystemEnabled) {
            List<AprilTagDetection> currentDetections = aprilTag.getFreshDetections();

            // Step through the list of detections see if the desired goal is visible
            if (currentDetections != null) {
                for (AprilTagDetection detection : currentDetections) {

                    if ((detection != null) && (detection.metadata != null) && (detection.metadata.id == targetTagID)) {
                        target = new Target(detection.ftcPose.range, detection.ftcPose.bearing);

                        if (showTelemetry) {
                            if (detection.metadata != null) {
                                myOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                                myOpMode.telemetry.addLine(String.format("Range %6.1f in, Bearing %6.1f deg.", target.range, target.bearing));
                            }
                        }
                    }
                }
            }
        }

        return target;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480 ))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();
    }

    public boolean cameraReady() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null)  return true;

        // Return camera ready status
        return (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    public void waitForCamera() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null)  return;

        // Wait for the camera to be open
        while (!myOpMode.isStopRequested() && !cameraReady()) {
            myOpMode.telemetry.addLine("Waiting for Camera");
            myOpMode.telemetry.update();
        }
    }

    /**
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */
    private void  setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null)  return;

        // Wait for the camera to be open
        waitForCamera();

        if (cameraReady()){
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpMode.sleep(20);
        }
    }

    /**
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) return;

        // Wait for the camera to be open
        waitForCamera();

        // Get camera control values unless we are stopping.
        if (cameraReady()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            maxGain = gainControl.getMaxGain();
        }
    }
}
