package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.vision.opencv.PredominantColorProcessor.Swatch.ARTIFACT_GREEN;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.auxtools.SubsystemBase;
import org.firstinspires.ftc.teamcode.auxtools.Target;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionSubsystem extends SubsystemBase {
    private final int OBELESK_TO_PATTERN_ID = 21;

    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    PredominantColorProcessor colorSensor;           // used for reading Artifact color

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
        initProcessors();

        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getCameraSetting();
        myExposure =  Math.min(3, minExposure);
        myGain     =  20;
        //setManualExposure(myExposure, myGain);
    }

    public int getPatternId() {
        int patternid = -1;
        ElapsedTime timer = new ElapsedTime();
        if (subsystemEnabled){
            while ((patternid < 0) && (timer.time() < 2.0)) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                if (currentDetections != null) {
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection != null) && (detection.metadata != null)){
                            patternid = detection.metadata.id - OBELESK_TO_PATTERN_ID;
                            break;
                        }
                    }
                }
            }

            // Switch to Color Sensing
            visionPortal.setProcessorEnabled(aprilTag, false);
            visionPortal.setProcessorEnabled(colorSensor, true);
        }

        if (patternid >= 0) {
            return patternid;
        } else {
            return 2;  // fastest
        }
    }

    public ArtifactColor getColor() {
        if (subsystemEnabled) {
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
            if (result.closestSwatch == ARTIFACT_GREEN) {
                return ArtifactColor.GREEN;
            } else {
                return ArtifactColor.PURPLE;
            }
        } else {
            return ArtifactColor.PURPLE;
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initProcessors() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        colorSensor = new PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(0.2, 0.9, 0.7, 0.5))
            .setSwatches(
                    PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                    PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                    PredominantColorProcessor.Swatch.RED,
                    PredominantColorProcessor.Swatch.BLUE,
                    PredominantColorProcessor.Swatch.YELLOW,
                    PredominantColorProcessor.Swatch.BLACK,
                    PredominantColorProcessor.Swatch.WHITE)
            .build();


        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(800, 600 ))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .addProcessor(colorSensor)
                .build();

        visionPortal.setProcessorEnabled(aprilTag, true);
        visionPortal.setProcessorEnabled(colorSensor, true);
    }

    public void disableProcessing() {
        if (isEnabled()) {
            visionPortal.setProcessorEnabled(aprilTag, false);
            visionPortal.setProcessorEnabled(colorSensor, false);
        }
    }

    public void enableTeleopProcessing() {
        if (isEnabled()) {
            visionPortal.setProcessorEnabled(aprilTag, false);
            visionPortal.setProcessorEnabled(colorSensor, true);
        }
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
