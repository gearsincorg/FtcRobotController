/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Autonomous opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
//import com.vuforia.CameraDevice;

@Autonomous(name="G-FORCE Autonomous", group="!Competition")
public class GFORCE_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig autoConfig = new AutoConfig();
    public GFORCE_Hardware robot = new GFORCE_Hardware();
    // public GFORCE_Navigation   nav           = new GFORCE_Navigation();

    public static final String TAG = "G-FORCE";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_QUAD_ELEMENT = "Quad";
    private static final String LABEL_SINGLE_ELEMENT = "Single";
    private TFObjectDetector tfod;

    private ElapsedTime autoTime = new ElapsedTime();

    boolean isRed;
    int ringsStacked = 0;

    public VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        autoConfig.init(hardwareMap.appContext, this);
        robot.init(this);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            autoConfig.init_loop(); //Run menu system
            sleep(20);
            findRings();


            // Get alliance color from Menu
            isRed = autoConfig.autoOptions.redAlliance;

            if (autoConfig.autoOptions.redAlliance) {
                robot.allianceColor = GFORCE_Hardware.AllianceColor.RED;
            } else {
                robot.allianceColor = GFORCE_Hardware.AllianceColor.BLUE;
            }

            //Starting autonomous reset heading to zero
            robot.resetHeading();
            robot.readSensors();
            autoTime.reset();
        }


        if (autoConfig.autoOptions.enabled) {
            // Testing code
            robot.driveAxialVelocity(450,0,700,2);
            robot.turnToHeading(-45,1);
            robot.sleepAndHoldHeading(-45, 3);
            findRings();

            switch (ringsStacked) {
                case 0:
                default:
                    robot.turnToHeading(0,1);
                    robot.sleepAndHoldHeading(0,1);
                    robot.driveAxialVelocity(1166,0,900,3);
                    break;
                case 1:
                    robot.turnToHeading(-17,1);
                    robot.sleepAndHoldHeading(-17,1);
                    robot.driveAxialVelocity(1824,-17,900,4);
                    break;
                case 4:
                    robot.turnToHeading(0,1);
                    robot.sleepAndHoldHeading(0,1);
                    robot.driveAxialVelocity(2390,0,900,6);
                    break;
            }

            robot.dropWobbleGoal();
        }

        robot.stopRobot();
        //Shut down TensorFlow before the robot runs
        if (tfod != null) {
            tfod.shutdown();
        }
    }


    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD_ELEMENT, LABEL_SINGLE_ELEMENT);
    }

    private int findRings() {
        ringsStacked = -1;

        if (tfod != null) {
           // getUpdatedRecognitions() will return null if no new information is available since
           // the last time that call was made.
           List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
           if (updatedRecognitions != null) {
               telemetry.addData("# Object Detected", updatedRecognitions.size());
               // step through the list of recognitions and display boundary info.
               int i = 0;
               for (Recognition recognition : updatedRecognitions) {
                   telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                   telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                           recognition.getLeft(), recognition.getTop());
                   telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                           recognition.getRight(), recognition.getBottom());
                   i++;

                   if (recognition.getLabel() == LABEL_QUAD_ELEMENT) {
                       ringsStacked = 4;
                   } else if (recognition.getLabel() == LABEL_SINGLE_ELEMENT) {
                       ringsStacked = 1;
                   }
               }

               telemetry.update();

           } else {
               ringsStacked = 0;
           }
        }
        return ringsStacked;
   }

}