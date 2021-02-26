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

@Autonomous(name="G-FORCE AUTO", group="!Competition", preselectTeleOp="G-FORCE TELEOP")
public class GFORCE_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig       autoConfig  = new AutoConfig();
    public GFORCE_Hardware  robot       = new GFORCE_Hardware();
    public GFORCE_Vision    vision      = new GFORCE_Vision();

    public  static final String TAG = "G-FORCE";
    private ElapsedTime autoTime    = new ElapsedTime();
    private int     ringsStacked    = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        autoConfig.init(hardwareMap.appContext, this);
        robot.init(this, vision);           // Motors, servos and Sensors.
        robot.startTiltCalibration();
        vision.init(this, robot);          // Basic Vision connection

        vision.initVuforia(false);   // Vuforia
        vision.activateVuforiaTargets(false);
        robot.endTiltCalibration();
        vision.initTFOD(true);       // Tensor flow
        vision.activateTFOD();               // Object Detaction

        // Wait for the game to start (driver presses PLAY)
        while (!opModeIsActive() && !isStopRequested()) {
            robot.readSensors();

            // display TFOD targets if Pilot Left Bumper pushed.
            if (gamepad1.left_bumper) {
                findRings();  // Wait 2 seconds while searching for a valid ring stack
            } else {
                telemetry.addData("---", "Press Left Bumper for TFOD");
                autoConfig.init_loop(); //Run menu system
            }

            sleep(10);
        }

        // Get alliance color from Menu
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

            // Wait for the required delay, unless we ar taking the spare action (double wobble)
            if (!autoConfig.autoOptions.spare) {
                robot.delayWithSound(autoConfig.autoOptions.delayInSec);
            }

            driveToLine();
            if (autoConfig.autoOptions.scoreRingsHigh) {
                shootHighGoal();
            } else if (autoConfig.autoOptions.scorePowerShot) {
                shootPowerShot();
            }

            if (autoConfig.autoOptions.scoreWobble) {
                scoreWobbleGoal();
            } else {
                // just drive forward to park
                if (autoConfig.autoOptions.park)
                    robot.driveAxialVelocity(250, 0, 300, 2);
            }
        }
        robot.stopRobot();
        vision.shutdownTFOD();
    }

    private int findRings() {
        ringsStacked = 0;

        autoTime.reset();
        if (vision.TFODIsValid()) {

           // getUpdatedRecognitions() will return null if no new information is available since
           // the last time that call was made.
           while (!isStopRequested() && (autoTime.time() < 2.0)) {
               List<Recognition> updatedRecognitions = vision.tfod.getUpdatedRecognitions();
               if (updatedRecognitions != null) {
                   telemetry.addData("# Object Detected =====================", updatedRecognitions.size());
                   // step through the list of recognitions and display boundary info.
                   double maxConfidence = -1.0;
                   for (Recognition recognition : updatedRecognitions) {
                       double confidence = recognition.getConfidence();
                       telemetry.addData("Target", String.format("%s %.2f", recognition.getLabel(), confidence));

                       // Keep looking for the best one;
                       if (confidence > maxConfidence) {
                           if (recognition.getLabel() == vision.LABEL_QUAD_ELEMENT) {
                               ringsStacked = 4;
                           } else if (recognition.getLabel() == vision.LABEL_SINGLE_ELEMENT) {
                               ringsStacked = 1;
                           }
                           maxConfidence = confidence;
                       }
                   }
                   break;  // exit out of while loop early.
               } else {
                   telemetry.addData("targets", "None");
               }
               telemetry.update();
           }
        }
        return ringsStacked;
    }

    private void driveToLine () {
        double goalHeading = autoConfig.autoOptions.startCenter ? 34 : -39;
        double runDistance = autoConfig.autoOptions.startCenter ? 1130 : 1110;

        if (autoConfig.autoOptions.scoreWobble) {
            // go and look at ring stack;
            robot.grabWobbleGoal();
            robot.sleepAndHoldHeading(0, 1);
            robot.driveAxialVelocity(200, 0, 700, 2);

            // look at ring stack
            robot.turnToHeading(goalHeading, 1.5);
            robot.sleepAndHoldHeading(goalHeading, 1);
            findRings();
            robot.turnToHeading(0, 1);
            robot.sleepAndHoldHeading(0, 1);

            // See if we can pickup the other goal
            if (autoConfig.autoOptions.spare) {
                // Wait for the required delay, unless we ar taking the spare action (double wobble)
                robot.delayWithSound(autoConfig.autoOptions.delayInSec);
            }

            robot.driveAxialVelocity(runDistance, 0, 900, 4);
        } else {
            // drive out to edge of launch zone to attempt shots
            robot.driveAxialVelocity(runDistance + 200, 0, 700, 2);
        }
    }

    private void shootPowerShot() {
        double goalHeading = autoConfig.autoOptions.startCenter ? 5  : -27;
        robot.releaseRings();
        robot.setSpinnerTarget(autoConfig.autoOptions.startCenter ? Target.CENTER_POWER_SHOT : Target.WALL_POWER_SHOT);

        robot.releaseWobbleGoal();
        robot.driveAxialVelocity(200,0,450,2);
        robot.sleepAndHoldHeading(goalHeading,0.25);
        robot.driveAxialVelocity(200, 0,-450,2);

        robot.runSpinners();
        robot.turnOnTiltPID();
        robot.sleepAndHoldHeading(goalHeading,2);

        robot.takeOneShot();
        if(!autoConfig.autoOptions.startCenter) {
            robot.jogSpinnerUp();
        }
        robot.sleepAndHoldHeading(goalHeading - 5,1);
        robot.takeOneShot();
        if(!autoConfig.autoOptions.startCenter) {
            robot.jogSpinnerUp();
        }
        robot.sleepAndHoldHeading(goalHeading - 9,1);
        robot.takeOneShot();
        robot.stopSpinners();
        robot.turnOffTiltPID();
        robot.turnToHeading(0,1);

        if (autoConfig.autoOptions.startCenter || ringsStacked > 0) {
            robot.driveAxialVelocity(250, 0, 300, 2);
            robot.grabWobbleGoal();
            robot.sleepAndHoldHeading(goalHeading, 0.75);
            robot.driveAxialVelocity(250, 0, -300, 2);
        }
    }

    private void shootHighGoal() {
        double goalHeading = autoConfig.autoOptions.startCenter ? 14  : -17.5; //18 : -19
        robot.releaseRings();
        robot.setSpinnerTarget(Target.HIGH_GOAL_AUTO);
        robot.runSpinners();

        robot.turnToTarget(2, true, goalHeading);  // Adjust speed based on range
        robot.releaseWobbleGoal();
        goalHeading = robot.getHeading();

        // Push the wobble goal forward
        robot.turnOnTiltPID();
        robot.driveAxialVelocity(200,goalHeading,450,2);
        robot.sleepAndHoldHeading(goalHeading,0.25);

        robot.driveAxialVelocity(200,goalHeading,-450,2);
        robot.runShooterFeeder(4.0);
        robot.runCollectors(0);
        robot.stopSpinners();
        robot.turnOffTiltPID();
        robot.driveAxialVelocity(250,goalHeading,300,2);
        robot.grabWobbleGoal();
        robot.sleepAndHoldHeading(goalHeading,0.75);
        robot.driveAxialVelocity(250,goalHeading,-300,2);
        // robot.turnToHeading(0,1);
    }

    private void scoreWobbleGoal() {
        if (autoConfig.autoOptions.startCenter) {
            switch (ringsStacked) {
                case 0:
                default:
                    placeWobbleGoal(900, 70, 700, 4);
                    break;
                case 1:
                    placeWobbleGoal(900, 30, 700, 3);
                    break;
                case 4:
                    placeWobbleGoal(1500, 35, 900, 4);
                    break;

            }

        } else {
            switch (ringsStacked) {
                case 0:
                default:
                    placeWobbleGoal(190, 5, 200, 2);
                    break;
                case 1:
                    placeWobbleGoal(850, -27, 700, 5);
                    break;
                case 4:
                    placeWobbleGoal(1200, 0, 900, 6);
                    break;
            }
        }
    }

    private void placeWobbleGoal(double distance, double heading, double speed, double timeout) {

        robot.turnToHeading(heading, 0.5);

        // slow down if this is a single wobble
        if (!autoConfig.autoOptions.startCenter && !autoConfig.autoOptions.spare && (ringsStacked > 0)) {
            speed = 300;
        }

        robot.driveAxialVelocity(distance, heading, speed, timeout);
        robot.releaseWobbleGoal();
        robot.sleepAndHoldHeading(heading,0.5);

        if (autoConfig.autoOptions.park) {
            if (ringsStacked == 0) {
                // Just backoff the minimum
                if (autoConfig.autoOptions.startCenter) {
                    robot.driveAxialVelocity(distance, 90, -600, timeout);
                } else {
                    robot.enableRingDrop();
                    robot.driveAxialVelocity(100,heading,-300,timeout);
                }

            } else {
                // roll back over starting location
                robot.driveAxialVelocity(distance - 400, heading, -900, timeout);
            }

            robot.grabWobbleGoal();
        } else {
            robot.driveAxialVelocity(150, heading, -300,1);
        }
    }
}