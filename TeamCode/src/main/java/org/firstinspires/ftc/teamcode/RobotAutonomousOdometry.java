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
    private Drive drive = new Drive(this);

    @Override public void runOpMode()
    {
        // Initialize the drive hardware
        drive.init();
        drive.showTelemetry(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();
        waitForStart();

        if (opModeIsActive())
        {
            drive.resetHeading();

            // Drive a triangle
            drive.driveAxial(65, 0.10);
            //drive.driveLateral(48, 0.5);
            //drive.turnToHeading(45, 0.25);
            //drive.driveAxial(48 * 1.41, 0.5);
            //drive.turnToHeading(0, 0.25);

            // hold robot and show telemetry.
            drive.holdLastHeading(4000);
        }
    }
}
