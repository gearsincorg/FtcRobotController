/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external Drive class that manages all hardware interactions.
 */

@TeleOp(name="Teleop Odometry", group = "Concept")
public class RobotTeleopOdometry extends LinearOpMode
{
    // get an instance of the "Drive" class.
    private Drive drive = new Drive(this);

    @Override public void runOpMode()
    {
        // Initialize the drive hardware
        drive.init();
        drive.showTelemetry(true);

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play drive");
        telemetry.update();

        waitForStart();
        drive.resetOdometry();

        while (opModeIsActive())
        {
            drive.readSensors();
            drive.moveRobot(-gamepad1.left_stick_y * 0.75, -gamepad1.left_stick_x * 0.75, -gamepad1.right_stick_x * 0.50);
        }
    }
}
