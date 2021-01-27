/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Autonomous opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;
//import com.vuforia.CameraDevice;

@Autonomous(name="G-FORCE AUTO", group="!Competition", preselectTeleOp="G-FORCE TELEOP")
public class GFORCE_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig       autoConfig  = new AutoConfig();
    public GFORCE_Hardware  robot       = new GFORCE_Hardware();
    public GFORCE_Vision    vision      = new GFORCE_Vision();

    public static final String TAG = "G-FORCE";

    private ElapsedTime autoTime = new ElapsedTime();

    boolean isRed;
    int ringsStacked = 0;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        autoConfig.init(hardwareMap.appContext, this);
        robot.init(this);
        vision.init(this);
        vision.initVuforia(false);
        vision.initTFOD(true);
        vision.activateTFOD();

        // Wait for the game to start (driver presses PLAY)
        while (!opModeIsActive() && !isStopRequested()) {
            sleep(20);

            // display TFOD targets if Pilot Left Bumper pushed.
            if (gamepad1.left_bumper) {
                findRings();
            } else {
                telemetry.addData("---", "Press Left Bumper for TFOD");
                autoConfig.init_loop(); //Run menu system
            }

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

        if (opModeIsActive() && autoConfig.autoOptions.enabled) {
            // Testing code
            robot.grabWobbleGoal();
            robot.sleepAndHoldHeading(0,1);
            robot.driveAxialVelocity(300,0,700,2);
            robot.turnToHeading(-45,1);
            robot.sleepAndHoldHeading(-45, 3);
            findRings();

            switch (ringsStacked) {
                case 0:
                default:
                    robot.turnToHeading(0,1);
                    robot.sleepAndHoldHeading(0,1);
                    robot.driveAxialVelocity(1316,0,900,3);
                    break;
                case 1:
                    robot.turnToHeading(-17,1);
                    robot.sleepAndHoldHeading(-17,1);
                    robot.driveAxialVelocity(1974,-17,900,4);
                    break;
                case 4:
                    robot.turnToHeading(0,1);
                    robot.sleepAndHoldHeading(0,1);
                    robot.driveAxialVelocity(2540,0,900,6);
                    break;
            }

            robot.releaseWobbleGoal();
            sleep(500);
            if(ringsStacked == 1) {
                robot.driveAxialVelocity(400,-17,-300,2);
                robot.turnToHeading(-90,1);
                robot.driveAxialVelocity(400,-90,-600,3);
                robot.turnToHeading(0,1);
                robot.sleepAndHoldHeading(0,1);
            } else {
                robot.driveAxialVelocity(120,0,-300,1);
            }
        }

        robot.stopRobot();
        vision.shutdownTFOD();
    }

    private int findRings() {
        ringsStacked = -1;

        if (vision.TFODIsValid()) {
           // getUpdatedRecognitions() will return null if no new information is available since
           // the last time that call was made.
           List<Recognition> updatedRecognitions = vision.getUpdatedRecognitions();
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

                   if (recognition.getLabel() == vision.LABEL_QUAD_ELEMENT) {
                       ringsStacked = 4;
                   } else if (recognition.getLabel() == vision.LABEL_SINGLE_ELEMENT) {
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