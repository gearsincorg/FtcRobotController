/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Autonomous opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
//import com.vuforia.CameraDevice;

@Autonomous(name="G-FORCE Autonomous", group="!Competition")
public class GFORCE_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    public AutoConfig   autoConfig    = new AutoConfig();
    public GFORCE_Hardware     robot         = new GFORCE_Hardware();
   // public GFORCE_Navigation   nav           = new GFORCE_Navigation();

    public static final String TAG = "G-FORCE";

    private ElapsedTime autoTime = new ElapsedTime();

    boolean isRed;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables.
        autoConfig.init(hardwareMap.appContext, this);
        robot.init(this);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();
        while (!opModeIsActive() && !isStopRequested()) {
            autoConfig.init_loop(); //Run menu system
            sleep(20);
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

        if (autoConfig.autoOptions.enabled) {
            // Testing code
            for (double head = 0; head < 360; head += 90) {
                robot.driveAxialVelocity(1500, head, 1000, 8);
                robot.turnToHeading(head + 90, 3);
                robot.sleepAndHoldHeading(head + 90, 0.5);
            }
        }

        robot.stopRobot();
    }

}