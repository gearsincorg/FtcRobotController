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

    private final double FRONT_SCORE_RANGE = 480.0;

    // get an instance of the "Robot" class.
    Robot robot = Robot.getInstance();
    Manipulator arm = new Manipulator(this);

    @Override public void runOpMode()
    {
        Globals.IS_AUTO = true;

        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(this, arm, true);
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
        sleep(500);

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            robot.resetHeading();
            arm.setLiftSetpoint(Manipulator.LIFT_AUTO_ANGLE);
            // Drive a path and return to start.
            robot.drive(28, 0.45, 0.20);
            robot.turnTo(90, 0.35, 0.20);
            arm.gotoFrontScore();
            robot.drive(35, 0.45, 0.0);
            robot.drive(2, 0.2, 0.20);
            arm.openRightGrabber();
            arm.runArmControl(1);
            robot.drive(-4, 0.25, 0.20);
            arm.gotoHome();


        }
    }
}
