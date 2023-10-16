/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external Drive class that manages all hardware interactions.
 */

@Autonomous(name="Autonomous Odometry", group = "Concept")
public class RobotAutonomousOdometry extends LinearOpMode
{
    // get an instance of the "Drive" class.
    private Robot robot = new Robot(this);

    @Override public void runOpMode()
    {
        // Initialize the drive hardware
        robot.init();
        robot.showTelemetry(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();
        waitForStart();

        if (opModeIsActive())
        {
            robot.resetHeading();

            // Drive a triangle
            robot.drive(48, 0.10, 0.25);
            robot.strafe(24, 0.10, 0.25);
            robot.strafe(-24, 0.10, 0.25);
            robot.drive(-48, 0.10, 0.25);
            //drive.turnToHeading(45, 0.25);
            //drive.driveAxial(48 * 1.41, 0.5);
            //drive.turnToHeading(0, 0.25);
        }
    }
}
