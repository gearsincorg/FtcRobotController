/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external Robot class that manages all hardware interactions.
 */

@Autonomous(name="Sample Autonomous", group = "Concept")
public class SampleAutonomous extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private Robot robot = new Robot(this);

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);
        robot.resetOdometry();
        robot.resetHeading();

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();
        waitForStart();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            robot.resetHeading();

            // Drive a path and return to start.
            robot.drive(48, 0.10, 0.20);
            robot.strafe(24, 0.10, 0.20);
            robot.turnToHeading(-90, 0.25, 0.20);
            robot.drive(24, 0.10, 0.20);
            robot.strafe(-48, 0.1, 0.20);
            robot.turnToHeading(0, 0.25, 0.20);
        }
    }
}
