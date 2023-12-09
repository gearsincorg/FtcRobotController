/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Alert;
import org.firstinspires.ftc.teamcode.subsystems.AlertState;
import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.TeamPropLocation;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external Drive class that manages all hardware interactions.
 */

@Autonomous(name="G-FORCE Autonomous", group = "AAA", preselectTeleOp="G-FORCE TELEOP")
public class GFORCE_Autonomous extends LinearOpMode
{
    // get an instance of the "Robot" class, then instanciate all other subsystems
    Robot           robot =  Robot.getInstance();
    Vision          vision = new Vision(this);
    Manipulator     arm =    new Manipulator(this);
    Alert           alert = new Alert(this);
    AutoConfig      autoConfig  = new AutoConfig(this);

    private ElapsedTime delayTime   = new ElapsedTime();  // User for delaying auto actions

    TeamPropLocation teamPropLocation = TeamPropLocation.UNKNOWN;

    @Override public void runOpMode()
    {
        Globals.IS_AUTO = true;

        // Initialize the robot hardware & Turn on telemetry
        robot.initialize(this, arm, vision, true);
        robot.resetOdometry();
        robot.resetHeading();

        vision.initialize(true);

        arm.initialize(false);
        arm.setRangeEnable(false);
        arm.homeArm();
        arm.wristToAutonomousPosition();

        // Turn on OpenCV Vision processing.
        vision.enableTeamProp();

        alert.initialize(false);
        autoConfig.initialize();

        // Loop while waiting for match to start;
        while(opModeInInit()) {

            autoConfig.runMenuUI(); //Run menu system

            alert.update();     // Update LED status
            telemetry.addLine("\n");

            // Set GLOBAL flags based on menu choices.
            if (autoConfig.autoOptions.redAlliance )
                Globals.ALLIANCE_COLOR = AllianceColor.RED;
            else
                Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

            Globals.PURPLE_PIXEL_ON_RIGHT =  ((autoConfig.autoOptions.redAlliance && autoConfig.autoOptions.startFront) ||
                    (!autoConfig.autoOptions.redAlliance && !autoConfig.autoOptions.startFront));

            // Update additional sensor.
            arm.readSensors();
            arm.runManualGrippers();

            TeamPropLocation locationTest = vision.getTeamPropLocation();
            if (locationTest != TeamPropLocation.UNKNOWN) {
                teamPropLocation = locationTest;
            }

            telemetry.addData("Last Detection", teamPropLocation);
            telemetry.addLine("\n");
            vision.telemetryTeamProp();

            // Alert the driver to the condition of the robot
            if (vision.getContourCount() == 0) {
                alert.setState(AlertState.VIDEO_ERROR);
            } else {
                alert.setState(AlertState.AUTO_PIXEL);
            }

            telemetry.update();
        }

        // Run Auto if stop was not pressed.
        if (opModeIsActive() && !autoConfig.autoOptions.disabled)
        {
            // Grab preload pixels and start auto
            arm.closeRightGrabber();
            arm.closeLeftGrabber();

            vision.enableAprilTag();

            // Delay the start of motion if requested by settings
            delay(autoConfig.autoOptions.delayStart);

            // prepare to move.
            robot.resetHeading();

            // Where is the robot starting on the field?
            if (autoConfig.autoOptions.startFront) {
                // Robot is starting at the front of the field.

                if (((Globals.ALLIANCE_COLOR == AllianceColor.BLUE) && (teamPropLocation == TeamPropLocation.LEFT_SIDE)) ||
                    ((Globals.ALLIANCE_COLOR == AllianceColor.RED)  && (teamPropLocation == TeamPropLocation.RIGHT_SIDE))) {
                    // Purple near Spike
                    robot.drive(18, 0.45, 0.0);
                    arm.setLiftSetpoint(5);
                    arm.setExtendSetpoint(5);
                    robot.turnTo(45, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.openPurpleGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);
                    robot.drive(32, 0.45, 0.0);
                    robot.turnTo(90, 0.35, 0);

                    if (!autoConfig.autoOptions.skipYellow) {
                        robot.drive(72, 0.45, 0);
                        delay(autoConfig.autoOptions.delayYellow);
                        arm.gotoFrontScore();
                        robot.strafe(28, 0.45, 0.0);
                        robot.driveToTag(1);
                        arm.waitTillArmInPosition();
                        arm.openGrabbers();
                        arm.runArmControl(1);
                        robot.drive(-4, 0.5, 0);
                        arm.gotoHome();
                        robot.turnTo(-90,0.35,0);
                        if (autoConfig.autoOptions.parkCenter) {
                            robot.strafe(30, 0.45, 0.0);
                        } else {
                            robot.strafe(-18, 0.45, 0.0);
                        }
                        robot.drive(-16, 0.45, 0);
                    } else {
                        arm.gotoHome();
                    }

                } else if (teamPropLocation == TeamPropLocation.CENTER) {
                    robot.drive(23, 0.45, 0.0);
                    robot.turnTo(90, 0.35, 0);
                    arm.setLiftSetpoint(7);
                    arm.setExtendSetpoint(3);
                    robot.drive(-16, 0.45, 0.0);
                    robot.strafe(-17, 0.45, 0.0);
                    arm.waitTillArmInPosition();
                    arm.openPurpleGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.strafe(-11, 0.45, 0.0);
                    if (!autoConfig.autoOptions.skipYellow) {
                        robot.drive(86, 0.45, 0);
                        delay(autoConfig.autoOptions.delayYellow);
                        arm.gotoFrontScore();
                        robot.strafe(24, 0.45, 0.0);
                        robot.driveToTag(2);
                        arm.waitTillArmInPosition();
                        arm.openGrabbers();
                        arm.runArmControl(1);
                        robot.drive(-4, 0.5, 0);
                        arm.gotoHome();
                        robot.turnTo(-90,0.35,0);
                        if (autoConfig.autoOptions.parkCenter) {
                            robot.strafe(24, 0.45, 0.0);
                        } else {
                            robot.strafe(-24, 0.45, 0.0);
                        }
                        robot.drive(-16, 0.45, 0);
                    } else {
                        arm.gotoHome();
                    }

                } else if (((Globals.ALLIANCE_COLOR == AllianceColor.BLUE) && (teamPropLocation == TeamPropLocation.RIGHT_SIDE)) ||
                           ((Globals.ALLIANCE_COLOR == AllianceColor.RED)  && (teamPropLocation == TeamPropLocation.LEFT_SIDE))) {
                    // Purple away from Spike
                    robot.drive(10, 0.45, 0.0);
                    arm.setLiftSetpoint(7);
                    arm.setExtendSetpoint(11);
                    robot.turnTo(-40, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.openPurpleGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);
                    robot.drive(40, 0.45, 0.0);
                    robot.turnTo(90, 0.35, 0);
                    if (!autoConfig.autoOptions.skipYellow) {
                        robot.drive(72, 0.45, 0);
                        delay(autoConfig.autoOptions.delayYellow);
                        arm.gotoFrontScore();
                        robot.strafe(18, 0.45, 0.0);
                        robot.driveToTag(3);
                        arm.waitTillArmInPosition();
                        arm.openGrabbers();
                        arm.runArmControl(1);
                        robot.drive(-4, 0.5, 0);
                        arm.gotoHome();
                        robot.turnTo(-90,0.35,0);
                        if (autoConfig.autoOptions.parkCenter) {
                            robot.strafe(18, 0.45, 0.0);
                        } else {
                            robot.strafe(-30, 0.45, 0.0);
                        }
                        robot.drive(-16, 0.45, 0);
                    } else {
                        arm.gotoHome();
                    }
                }
            } else {
                // Robot is starting at the back of the field.

                if ( ((Globals.ALLIANCE_COLOR == AllianceColor.BLUE) && (teamPropLocation == TeamPropLocation.LEFT_SIDE )) ||
                     ((Globals.ALLIANCE_COLOR == AllianceColor.RED)  && (teamPropLocation == TeamPropLocation.RIGHT_SIDE)) ) {
                    // Purple away from Spike
                    robot.drive(10, 0.45, 0.0);
                    arm.setLiftSetpoint(7);
                    arm.setExtendSetpoint(11);
                    robot.turnTo(40, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.openPurpleGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);

                    if (!autoConfig.autoOptions.skipYellow) {
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.strafe(24, 0.45, 0.0);
                        robot.turnTo(90, 0.35, 0);
                        arm.gotoFrontScore();
                        robot.strafe(-12, 0.45, 0.0);
                        robot.driveToTag(1);
                        arm.waitTillArmInPosition();
                        arm.openGrabbers();
                        arm.runArmControl(1);
                        robot.drive(-4, 0.5, 0);
                        arm.gotoHome();
                        robot.turnTo(-90,0.35,0);
                        if (autoConfig.autoOptions.parkCenter) {
                            robot.strafe(30, 0.45, 0.0);
                        } else {
                            robot.strafe(-18, 0.45, 0.0);
                        }
                        robot.drive(-16, 0.45, 0);
                    } else {
                        arm.gotoHome();
                    }

                } else if (teamPropLocation == TeamPropLocation.CENTER) {
                    robot.drive(23, 0.45, 0.0);
                    arm.setLiftSetpoint(7);
                    arm.setExtendSetpoint(7);
                    arm.waitTillArmInPosition();
                    arm.openPurpleGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(90, 0.35, 0);
                    if (!autoConfig.autoOptions.skipYellow) {
                        delay(autoConfig.autoOptions.delayYellow);
                        arm.gotoFrontScore();
                        robot.drive(24, 0.5, 0);
                        robot.strafe(-6, 0.45, 0.0);
                        robot.driveToTag(2);
                        arm.waitTillArmInPosition();
                        arm.openGrabbers();
                        arm.runArmControl(1);
                        robot.drive(-4, 0.5, 0);
                        arm.gotoHome();
                        robot.turnTo(-90,0.35,0);
                        if (autoConfig.autoOptions.parkCenter) {
                            robot.strafe(24, 0.45, 0.0);
                        } else {
                            robot.strafe(-24, 0.45, 0.0);
                        }
                        robot.drive(-16, 0.45, 0);
                    } else {
                        arm.gotoHome();
                    }

                } else if (((Globals.ALLIANCE_COLOR == AllianceColor.BLUE) && (teamPropLocation == TeamPropLocation.RIGHT_SIDE)) ||
                        ((Globals.ALLIANCE_COLOR == AllianceColor.RED)  && (teamPropLocation == TeamPropLocation.LEFT_SIDE))) {
                    // Purple near Spike
                    robot.drive(18, 0.45, 0.0);
                    arm.setLiftSetpoint(5);
                    arm.setExtendSetpoint(5);
                    robot.turnTo(-45, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.openPurpleGrabber();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);
                    if (!autoConfig.autoOptions.skipYellow) {
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.strafe(24, 0.45, 0);
                        robot.turnTo(90, 0.35, 0);
                        arm.gotoFrontScore();
                        robot.strafe(-18, 0.45, 0.0);
                        robot.driveToTag(3);
                        arm.waitTillArmInPosition();
                        arm.openGrabbers();
                        arm.runArmControl(1);
                        robot.drive(-4, 0.5, 0);
                        arm.gotoHome();
                        robot.turnTo(-90,0.35,0);
                        if (autoConfig.autoOptions.parkCenter) {
                            robot.strafe(18, 0.45, 0.0);
                        } else {
                            robot.strafe(-30, 0.45, 0.0);
                        }
                        robot.drive(-16, 0.45, 0);
                    } else {
                        arm.gotoHome();
                    }
                }

            }
        }

        vision.disableAll();
        alert.setState(AlertState.OFF);
        arm.runArmControl(5);
    }

    void delay(double delaySec) {
        if (delaySec > 0) {
            delayTime.reset();
            while (opModeIsActive() && (delayTime.time() < delaySec)) {
                arm.runArmControl(0.25);
            }
        }
    }
}
