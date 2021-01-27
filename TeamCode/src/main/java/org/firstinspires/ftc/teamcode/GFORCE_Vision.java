package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class GFORCE_Vision {

    // Public Data  ========================================

    // VUFORIA variables
    public VuforiaLocalizer vuforia;

    // TENSOR FLOW object Detection variables
    public TFObjectDetector tfod;

    // Image Targeting
    public  double robotX;
    public  double robotY;
    public  double targetRange;
    public  double targetBearing;
    public  double robotBearing;
    public  double relativeBearing;
    public  OpenGLMatrix    robotLocation;
    public  OpenGLMatrix    lastLocation = null;
    public  boolean         targetVisible = false;

    // Constants
    private final float CAMERA_VERTICAL_DISPLACEMENT  = 200.0f;   // eg: Camera is 8 inches above ground
    private final float mmTargetHeight = 150.0f;   // the height in mm of the center of the target image above the floor

    public final String TFOD_MODEL_ASSET     = "UltimateGoal.tflite";
    public final String VUFORIA_MODEL_ASSET  = "UltimateGoal";
    public final String LABEL_QUAD_ELEMENT   = "Quad";
    public final String LABEL_SINGLE_ELEMENT = "Single";

    // Private data  ========================================
    private static LinearOpMode myOpMode = null;

    private static final String VUFORIA_KEY =
            "ASFl1ib/////AAABmdtl1FqwZUIEqtOW/F+xX70YsCPMRYbusW+Av5TpUTDuB3VJT4z6ju8tkAzSKLD0cIwdp/o/3ggJzx27+OsIHWn8OTNfsAtxIzQVSCa75gI76/v006khzWpGV1wmdoEgK7JkvEns6BCzmgfSBSThg70Ej42wDF7l5FuIXUhm/AAMJ7sHLlMl5BboZg/vRyNRFTbEbFLyj98DOwLlaNl9DvUtf5bGBOHwFCNOBX8vlxWVU3aZZpGNxNTX/KyZ84TWECIxg8SeRSz3QcBEwsBYX97HXfj4nJxn93u8m5SZmoHF11MPkV0tlqemRwrCy/MJ3eGB3WCJ+MEeCAYeVa30E+WEkVTiFQAo4WW3vKuEVuBc";

    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> allTrackables;

    private TFObjectDetector.Parameters tfodParameters = null;

    /* Constructor */
    public GFORCE_Vision() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode opMode) {
        // Save reference to Hardware map
        myOpMode = opMode;
        tfod = null;
    }

    /* =============================================================
    * Vuforia target tracking
    * ============================================================== */

    public void initVuforia(boolean preview) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        if (preview) {
            int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void activateVuforiaTargets(boolean preview) {
        //  Instantiate the Vuforia engine
        if (vuforia == null) {
            initVuforia(preview);
        }

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset(VUFORIA_MODEL_ASSET);
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower");

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(0, 0, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        for (VuforiaTrackable trackable : allTrackables) {
            trackable.setLocation(OpenGLMatrix
                    .translation(0, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            /**  Let all the trackable listeners know where the phone is.  */
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        }

        targetsUltimateGoal.activate();
    }

    public void deactivateVuforiaTargets() {
        targetsUltimateGoal.deactivate();
    }


    /***
     * Look for new target position and generate tracking data
     * @return
     */
    public boolean newTargetPosition(){

        // check all the trackable targets to see which one (if any) is visible.
        boolean newTargetFound  = false;
        targetVisible     = false;
        double lrobotX;
        double lrobotY;
        double ltargetRange;
        double ltargetBearing;
        double lrobotBearing;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                myOpMode.telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                    robotLocation = robotLocationTransform;
                    VectorF trans = robotLocation.getTranslation();
                    Orientation rot = Orientation.getOrientation(robotLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Robot position is defined by the standard Matrix translation (x and y)
                    lrobotX = trans.get(0);
                    lrobotY = trans.get(1);

                    // Robot bearing (in cartesian system) is defined by the standard Matrix z rotation
                    lrobotBearing = rot.thirdAngle;

                    // target range is based on distance from robot position to origin.
                    ltargetRange = Math.hypot(lrobotX, lrobotY);

                    // target bearing is based on angle formed between the X axis to the target range line
                    ltargetBearing = Math.toDegrees(-Math.asin(lrobotY / ltargetRange));

                    // sanity check
                    if ((Math.abs(lrobotBearing) < 30) && (Math.abs(ltargetBearing) < 30)) {
                        robotX = lrobotX;
                        robotY = lrobotY;
                        robotBearing = lrobotBearing;
                        targetRange = ltargetRange;
                        targetBearing = ltargetBearing;

                        // Target relative bearing is the target currentHeading relative to the direction the robot is pointing.
                        relativeBearing = targetBearing - robotBearing;
                        newTargetFound = true;
                    }
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            myOpMode.telemetry.addData("Robot", "X, Y (H) = %.0f, %.0f (%.1f)", robotX, robotY, robotBearing);
            myOpMode.telemetry.addData("Target", "R (B) (RB) = %.0f  (%.1f) (%.1f)", targetRange, targetBearing, relativeBearing);
        }
        else {
            myOpMode.telemetry.addData("Visible Target", "none");
        }
        return(newTargetFound);
    }

    /** ============================================================================
     * TensorFlow Object Detection engine.
     * ============================================================================ */
    public void initTFOD (boolean preview) {
        if(preview) {
            int tfodMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        } else {
            tfodParameters = new TFObjectDetector.Parameters();
        }

        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD_ELEMENT, LABEL_SINGLE_ELEMENT);
    }

    public void activateTFOD () {
        if (tfod != null)
            tfod.activate();
    }

    public void shutdownTFOD () {
        if (tfod != null) {
            tfod.deactivate();
            tfod.shutdown();
        }
    }

    public boolean TFODIsValid() {
        return (tfod != null);
    }

    public List<Recognition> getUpdatedRecognitions() {
        return tfod.getUpdatedRecognitions();
    }
}
