/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external Drive class that manages all hardware interactions.
 */

@Autonomous(name="G-FORCE Autonomous", group = "AAA")
public class GFORCE_Autonomous extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private Robot robot = new Robot(this);
    Manipulator arm = new Manipulator(this);

    @Override public void runOpMode()
    {
        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(true);
        robot.resetOdometry();
        robot.resetHeading();

        arm.initialize(true);
        arm.homeArm();
        arm.autoOpenGrabbers();

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();
        waitForStart();
        arm.closeRightGrabber();
        arm.closeLeftGrabber();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            robot.resetHeading();

            // Drive a path and return to start.
            robot.drive(28, 0.45, 0.20);
            robot.turnTo(90, 0.35, 0.20);
            robot.drive(24, 0.45, 0.20);

        }
    }
}
