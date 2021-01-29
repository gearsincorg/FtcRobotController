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
            robot.readSensors();

            // display TFOD targets if Pilot Left Bumper pushed.
            if (gamepad1.left_bumper) {
                findRings();
            } else {
                telemetry.addData("---", "Press Left Bumper for TFOD");
                autoConfig.init_loop(); //Run menu system
            }

            sleep(10);
        }

        // Get alliance color from Menu
        isRed = autoConfig.autoOptions.redAlliance;
        if (autoConfig.autoOptions.redAlliance) {
            robot.allianceColor = GFORCE_Hardware.AllianceColor.RED;
        } else {
            robot.allianceColor = GFORCE_Hardware.AllianceColor.BLUE;
        }

        //Starting autonomous. reset heading to zero
        robot.resetHeading();
        robot.readSensors();
        autoTime.reset();

        if (opModeIsActive() && autoConfig.autoOptions.enabled) {

            // Wait for the required delay
            robot.delayWithSound(autoConfig.autoOptions.delayInSec);

            driveToLine();
            if (autoConfig.autoOptions.scorePowerShot) {
                shootPowerShot();
            }
            if (autoConfig.autoOptions.scoreWobble) {
                placeWobbleGoal();
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
    private void driveToLine () {
        robot.grabWobbleGoal();
        robot.sleepAndHoldHeading(0,1);
        robot.driveAxialVelocity(300,0,700,2);

        double goalHeading = autoConfig.autoOptions.startCenter ? 35  : -35;
        robot.turnToHeading(goalHeading, 1);
        robot.sleepAndHoldHeading(goalHeading, 3);
        findRings();
        robot.turnToHeading(0,1);
        robot.sleepAndHoldHeading(0,1);
        robot.driveAxialVelocity(1000,0,900,2);
    }

    private void shootPowerShot() {
        double goalHeading = autoConfig.autoOptions.startCenter ? 0  : -30;
        robot.sleepAndHoldHeading(goalHeading,1);
        robot.releaseRings();
        double goalSpeed = autoConfig.autoOptions.startCenter ? robot.POWER_SHOT_SPEED : robot.HIGH_SHOOTER_SPEED;
        robot.spinnerAtSpeed(goalSpeed);
        robot.sleepAndHoldHeading(goalHeading,1);
        robot.runCollectors(1);
        robot.sleepAndHoldHeading(goalHeading - 5,2);
        robot.runCollectors(0);
        robot.runSpinners(0);
        robot.turnToHeading(0,1);

    }

    private void placeWobbleGoal () {
        if (autoConfig.autoOptions.startCenter) {
            switch (ringsStacked) {
                case 0:
                default:
                    robot.turnToHeading(70, 1);
                    robot.driveAxialVelocity(900, 70, 700, 1);
                    robot.releaseWobbleGoal();
                    robot.driveAxialVelocity(300,70,-300,1);
                    break;
                case 1:
                    robot.turnToHeading(30, 1);
                    robot.sleepAndHoldHeading(30, 1);
                    robot.driveAxialVelocity(900, 30, 700, 4);
                    robot.releaseWobbleGoal();
                    robot.driveAxialVelocity(300,30,-300,1);
                    break;
                case 4:
                    robot.turnToHeading(35, 1);
                    robot.sleepAndHoldHeading(35, 1);
                    robot.driveAxialVelocity(1600, 35, 900, 6);
                    robot.releaseWobbleGoal();
                    robot.driveAxialVelocity(300,35,-300,1);
                    break;

            }

        } else {
            switch (ringsStacked) {
                case 0:
                default:
                    robot.releaseWobbleGoal();
                    robot.sleepAndHoldHeading(0,0.5);
                    robot.driveAxialVelocity(300,0,-300,1);
                    break;
                case 1:
                    robot.turnToHeading(-30, 1);
                    robot.sleepAndHoldHeading(-30, 1);
                    robot.driveAxialVelocity(900, -30, 900, 4);
                    robot.releaseWobbleGoal();
                    robot.sleepAndHoldHeading(-30,0.5);
                    robot.driveAxialVelocity(300,-30,-300,1);
                    break;
                case 4:
                    robot.sleepAndHoldHeading(0, 1);
                    robot.driveAxialVelocity(1200, 0, 900, 6);
                    robot.releaseWobbleGoal();
                    robot.sleepAndHoldHeading(0,0.5);
                    robot.driveAxialVelocity(300,0,-300,1);
                    break;

            }


        }

    }

}