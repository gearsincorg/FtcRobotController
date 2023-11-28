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
        arm.setRangeEnable(false);
        arm.homeArm();
        arm.wristToPickupPosition();  //  FIX THIS... should be Auto Pickup position for grabbers.

        // Turn on OpenCV Vision processing.
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


        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Grab preload pixels and start auto
            arm.closeRightGrabber();
            arm.closeLeftGrabber();

            vision.enableAprilTag();
            robot.resetHeading();

            //code for front blue allliance start
            {// Drive a path and return to start.
                if(teamPropLocation == TeamPropLocation.LEFT_SIDE) {
                    robot.drive(18, 0.45, 0.0);
                    arm.setLiftSetpoint(5);
                    arm.setExtendSetpoint(5);
                    robot.turnTo(45, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.openLeftGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);
                    robot.drive(34, 0.45, 0.0);
                    robot.turnTo(90, 0.35, 0);
                    robot.drive(72, 0.45, 0);
                    arm.gotoFrontScore();
                    robot.strafe(28, 0.45, 0.0);
                    robot.driveToTag(1);
                    arm.waitTillArmInPosition();
                    arm.openGrabbers();
                    arm.runArmControl(1);
                    robot.drive(-4, 0.5, 0);
                    arm.gotoHome();
                    robot.strafe(16, 0.45, 0.0);

                } else if (teamPropLocation == TeamPropLocation.RIGHT_SIDE){
                    robot.drive(10, 0.45, 0.0);
                    arm.setLiftSetpoint(7);
                    arm.setExtendSetpoint(11);
                    robot.turnTo(-40, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.openLeftGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);
                    robot.drive(42, 0.45, 0.0);
                    robot.turnTo(90, 0.35, 0);
                    robot.drive(72, 0.45, 0);
                    arm.gotoFrontScore();
                    robot.strafe(16, 0.45, 0.0);
                    robot.driveToTag(3);
                    arm.waitTillArmInPosition();
                    arm.openGrabbers();
                    arm.runArmControl(1);
                    robot.drive(-4, 0.5, 0);
                    arm.gotoHome();
                    robot.strafe(32, 0.45, 0.0);

                } else if (teamPropLocation == TeamPropLocation.CENTER) {
                    robot.drive(21, 0.45, 0.0);
                    arm.setLiftSetpoint(7);
                    arm.setExtendSetpoint(3);
                    robot.turnTo(90, 0.35, 0);
                    robot.drive(-16, 0.45, 0.0);
                    robot.strafe(-18, 0.45, 0.0);
                    arm.waitTillArmInPosition();
                    arm.openLeftGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.strafe(-12, 0.45, 0.0);
                    robot.drive(86, 0.45, 0);
                    arm.gotoFrontScore();
                    robot.strafe(22, 0.45, 0.0);
                    robot.driveToTag(2);
                    arm.waitTillArmInPosition();
                    arm.openGrabbers();
                    arm.runArmControl(1);
                    robot.drive(-4, 0.5, 0);
                    arm.gotoHome();
                    robot.strafe(24, 0.45, 0.0);

                }


            }

        }
        vision.disableAll();
        arm.runArmControl(5);
    }
}
