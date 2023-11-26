/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.TeamPropLocation;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external Drive class that manages all hardware interactions.
 */

@Autonomous(name="G-FORCE Autonomous", group = "AAA")
public class GFORCE_Autonomous extends LinearOpMode
{
    // get an instance of the "Robot" class.
    Robot            robot =  Robot.getInstance();
    Manipulator      arm =    new Manipulator(this);
    Vision           vision = new Vision(this);
    TeamPropLocation teamPropLocation = TeamPropLocation.UNKNOWN;

    @Override public void runOpMode()
    {
        Globals.IS_AUTO = true;

        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(this, arm, vision, true);
        robot.resetOdometry();
        robot.resetHeading();

        vision.initialize(true);

        arm.initialize(true);
        arm.homeArm();
        arm.wristToPickupPosition();  //  FIX THIS... should be Auto Pickup position for grabbers.

        vision.enableTeamProp();

        // Loop while waiting for match to start;
        while(opModeInInit()) {
            arm.readSensors();
            arm.runManualGrippers();

            TeamPropLocation locationTest = vision.getTeamPropLocation();
            if (locationTest != TeamPropLocation.UNKNOWN) {
                teamPropLocation = locationTest;
            }

            telemetry.addData("Team Prop", teamPropLocation);
            telemetry.update();
        }

        // Grab preload pixels and start auto
        arm.closeRightGrabber();
        arm.closeLeftGrabber();

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            vision.enableAprilTag();
            robot.resetHeading();
            // Drive a path and return to start.
             robot.drive(28, 0.45, 0.20);
            robot.turnTo(90, 0.35, 0);
            arm.gotoFrontScore();
            arm.runArmControl(2);
            robot.driveToTag(9);
            arm.waitTillArmInPosition();
            arm.openGrabbers();
            arm.runArmControl(1);
            robot.drive(-4, 0.5, 0);
            arm.gotoHome();
        }
        vision.disableAll();
        arm.runArmControl(5);
    }
}
