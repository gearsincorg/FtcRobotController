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
 * This is the Autonomous OpMode for G-FORCE's Center Stage robot
 */

@Autonomous(name="G-FORCE AUTONOMOUS", group = "AAA", preselectTeleOp="G-FORCE TELEOP")
public class GFORCE_Autonomous extends LinearOpMode
{
    // get an instance of the "Robot" class, then instanciate all other subsystems
    Robot           robot =  Robot.getInstance();
    Vision          vision = new Vision(this);
    Manipulator     arm =    new Manipulator(this);
    Alert           alert = new Alert(this);
    AutoConfig      autoConfig  = new AutoConfig(this);

    private ElapsedTime delayTime   = new ElapsedTime();  // User for delaying auto actions

    // TeamPropLocation teamPropLocation = TeamPropLocation.UNKNOWN;

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
        arm.wristToAutonomousPosition();

        // Turn on OpenCV Vision processing.
        vision.enableTeamProp();

        alert.initialize(false);
        autoConfig.initialize();

        while(opModeInInit()) {

            autoConfig.runMenuUI(); //Run menu system

            alert.update();     // Update LED status
            telemetry.addLine("\n");

            // Set GLOBAL flags based on menu choices.
            if (autoConfig.autoOptions.redAlliance )
                Globals.ALLIANCE_COLOR = AllianceColor.RED;
            else
                Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

            // determine colored pixel locations
            Globals.PURPLE_PIXEL_ON_RIGHT =  ((autoConfig.autoOptions.redAlliance && autoConfig.autoOptions.startFront) ||
                    (!autoConfig.autoOptions.redAlliance && !autoConfig.autoOptions.startFront));

            // Update additional sensor.
            arm.readSensors();
            arm.runManualGrippers();

            TeamPropLocation locationTest = vision.getTeamPropLocation();
            if (locationTest != TeamPropLocation.UNKNOWN) {
                Globals.TEAM_PROP_LOCATION = locationTest;
            }

            telemetry.addData("Last Detection", Globals.TEAM_PROP_LOCATION);
            telemetry.addLine("\n");
            vision.telemetryTeamProp();

            // Alert the driver to the condition of the robot
            if ((vision.getContourCount() == 0) || (Math.abs(robot.pitch) > 10) ){
                alert.setState(AlertState.VIDEO_ERROR);
            } else {
                alert.setState(AlertState.AUTO_PIXEL);
            }

            telemetry.update();
        }

        // Run Auto if stop was not pressed.
        if (opModeIsActive())
        {
            // Grab preload pixels and start auto
            arm.grabLeftPixel();
            arm.grabRightPixel();

            vision.enableAprilTag();

            // Delay the start of motion if requested by settings
            delay(autoConfig.autoOptions.delayStart);

            // prepare to move.
            robot.resetHeading();

            // Where is the robot starting on the field?
            if (autoConfig.autoOptions.startFront) {
                // Robot is starting at the front of the field.
                Globals.PLACE_YELLOW_HIGH = true;

                if (((Globals.ALLIANCE_COLOR == AllianceColor.BLUE) && (Globals.TEAM_PROP_LOCATION == TeamPropLocation.LEFT_SIDE)) ||
                    ((Globals.ALLIANCE_COLOR == AllianceColor.RED)  && (Globals.TEAM_PROP_LOCATION == TeamPropLocation.RIGHT_SIDE))) {
                    //  ####
                    //  ####   FRONT :  SPIKE NEAR TRUSS   ########################################################################
                    //  ####
                    robot.drive(18, 0.5, 0.0, false);
                    arm.setLiftSetpoint(10);
                    arm.setExtendSetpoint(5);
                    robot.turnTo(45, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.dropPurplePixel();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);

                    if (autoConfig.autoOptions.whitePixel) {
                        robot.strafe(-10, 0.7, 0.0);
                        robot.turnTo(-90, 0.5, 0);
                        arm.setLiftSetpoint(Manipulator.LIFT_HOME_ANGLE);
                        robot.strafe(7, 0.7, 0.0);
                        robot.driveToStackTag();
                        arm.setLiftSetpoint(Manipulator.LIFT_STACK_LEVEL5);
                        arm.waitTillArmInPosition();
                        arm.runArmControl(0.25);

                        robot.drive(5, 0.35, 0.0, true);  // Grab White Pixel
                        arm.runArmControl(0.5);
                        robot.drive(-7, 0.5, 0.0, false);
                        robot.turnTo(90, 0.5, 0);
                        arm.gotoSafeDriving();
                        robot.strafe(-28, 0.75, 0.0);
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.drive(93, 0.7, 0, false);
                    } else {
                        arm.gotoSafeDriving();

                        robot.drive(32, 0.5, 0.0, false);
                        robot.turnTo(90, 0.5, 0);
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.drive(72, 0.7, 0, false);
                    }

                    arm.gotoFrontScore();
                    robot.strafe(28, 0.6, 0.0);
                    robot.driveToBackdropTag(1);
                    arm.waitTillArmInPosition();

                    if (autoConfig.autoOptions.whitePixel) {
                        arm.dropYellowPixel();
                        arm.runArmControl(0.5);
                        arm.dropPurplePixel();
                    } else {
                        arm.dropPixels();
                    }
                    arm.runArmControl(1);
                    robot.drive(-4, 0.5, 0, false);
                    arm.gotoHome();
                    robot.turnTo(-90,0.5,0);
                    if (autoConfig.autoOptions.parkCenter) {
                        robot.strafe(30, 0.7, 0.0);
                    } else {
                        robot.strafe(-18, 0.7, 0.0);
                    }
                    robot.drive(-16, 0.45, 0, false);

                } else if ((Globals.TEAM_PROP_LOCATION == TeamPropLocation.CENTER) || (Globals.TEAM_PROP_LOCATION == TeamPropLocation.UNKNOWN)) {
                    //  ####
                    //  ####   FRONT :  CENTER SPIKE   ########################################################################
                    //  ####

                    robot.drive(23, 0.5, 0.0, false);
                    arm.setLiftSetpoint(5);
                    arm.setExtendSetpoint(5);
                    arm.waitTillArmInPosition();
                    arm.dropPurplePixel();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    arm.waitTillArmInPosition();

                    if (autoConfig.autoOptions.whitePixel) {
                        robot.strafe(-10, 0.7, 0.0);
                        robot.turnTo(-90, 0.5, 0);
                        arm.setLiftSetpoint(Manipulator.LIFT_HOME_ANGLE);
                        robot.driveToStackTag();
                        arm.setLiftSetpoint(Manipulator.LIFT_STACK_LEVEL5);
                        arm.waitTillArmInPosition();
                        arm.runArmControl(0.25);

                        robot.drive(5, 0.35, 0.0, true);  // Grab White Pixel
                        arm.runArmControl(0.5);
                        robot.drive(-7, 0.5, 0.0, false);
                        arm.gotoSafeDriving();
                        robot.turnTo(90, 0.5, 0);
                        robot.strafe(-29, 0.75, 0.0);
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.drive(93, 0.7, 0, false);
                    } else {
                        arm.gotoSafeDriving();
                        robot.strafe(-20, 0.7, 0.0);
                        robot.drive(28, 0.7, 0.0, false);
                        robot.turnTo(90, 0.5, 0);
                        robot.drive(96, 0.7, 0, false);
                    }

                    arm.gotoFrontScore();
                    robot.strafe(24, 0.6, 0.0);
                    robot.driveToBackdropTag(2);
                    arm.waitTillArmInPosition();

                    if (autoConfig.autoOptions.whitePixel) {
                        arm.dropYellowPixel();
                        arm.runArmControl(0.5);
                        arm.dropPurplePixel();
                    } else {
                        arm.dropPixels();
                    }

                    arm.runArmControl(1);
                    robot.drive(-4, 0.5, 0, false);
                    arm.gotoHome();
                    robot.turnTo(-90,0.35,0);
                    if (autoConfig.autoOptions.parkCenter) {
                        robot.strafe(24, 0.5, 0.0);
                    } else {
                        robot.strafe(-24, 0.5, 0.0);
                    }
                    robot.drive(-16, 0.45, 0, false);

                } else if (((Globals.ALLIANCE_COLOR == AllianceColor.BLUE) && (Globals.TEAM_PROP_LOCATION == TeamPropLocation.RIGHT_SIDE)) ||
                           ((Globals.ALLIANCE_COLOR == AllianceColor.RED)  && (Globals.TEAM_PROP_LOCATION == TeamPropLocation.LEFT_SIDE))) {
                    //  ####
                    //  ####   FRONT :  SPIKE NEAR PERIMETER   ########################################################################
                    //  ####
                    robot.drive(10, 0.5, 0.0, false);
                    arm.setLiftSetpoint(10);
                    arm.setExtendSetpoint(11);
                    robot.turnTo(-40, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.dropPurplePixel();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);
                    arm.gotoSafeDriving();
                    robot.drive(40, 0.7, 0.0, false);
                    robot.turnTo(90, 0.5, 0);

                    delay(autoConfig.autoOptions.delayYellow);
                    robot.drive(72, 0.7, 0, false);
                    arm.gotoFrontScore();
                    robot.strafe(18, 0.7, 0.0);
                    robot.driveToBackdropTag(3);
                    arm.waitTillArmInPosition();
                    arm.dropPixels();
                    arm.runArmControl(1);
                    robot.drive(-4, 0.5, 0, false);
                    arm.gotoHome();
                    robot.turnTo(-90,0.5,0);
                    if (autoConfig.autoOptions.parkCenter) {
                        robot.strafe(18, 0.7, 0.0);
                    } else {
                        robot.strafe(-30, 0.7, 0.0);
                    }
                    robot.drive(-16, 0.7, 0, false);
                }
            } else {
                Globals.PLACE_YELLOW_HIGH = false;

                if ( ((Globals.ALLIANCE_COLOR == AllianceColor.BLUE) && (Globals.TEAM_PROP_LOCATION == TeamPropLocation.LEFT_SIDE )) ||
                     ((Globals.ALLIANCE_COLOR == AllianceColor.RED)  && (Globals.TEAM_PROP_LOCATION == TeamPropLocation.RIGHT_SIDE)) ) {
                    //  ####
                    //  ####   REAR :  SPIKE NEAR BACKDROP  ########################################################################
                    //  ####
                    robot.drive(10, 0.45, 0.0, false);
                    arm.setLiftSetpoint(10);
                    arm.setExtendSetpoint(11);
                    robot.turnTo(40, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.dropPurplePixel();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);

                    if (autoConfig.autoOptions.doubleYellow) {
                        robot.drive(-7, 0.7, 0.1, false);
                        arm.waitTillArmInPosition();  // should already be there.
                        robot.turnTo(-90, 0.5, 0.25);
                        //arm.wristToHome();
                        arm.dropPurplePixel();
                        arm.setLiftSetpoint(Manipulator.LIFT_HOME_ANGLE);
                        robot.drive(27, 0.5, 0.0, false);
                        robot.drive(6, 0.25, 0.0, true);   // Grab Pixel
                        arm.runArmControl(0.5);
                        arm.gotoSafeDriving();
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.drive(-57, 0.7, 0.0, false);
                        robot.strafe(18, 0.7, 0.0);
                        arm.gotoFrontScore();
                        robot.turnTo(90, 0.5, 0);
                    } else {
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.strafe(24, 0.7, 0.0);
                        robot.turnTo(90, 0.5, 0);
                        arm.gotoFrontScore();
                        robot.strafe(-12, 0.7, 0.0);
                    }

                    robot.driveToBackdropTag(1);
                    arm.waitTillArmInPosition();
                    arm.dropPixels();
                    arm.runArmControl(1);

                    robot.drive(-4, 0.7, 0, false);
                    arm.gotoHome();
                    robot.turnTo(-90,0.5,0);
                    if (autoConfig.autoOptions.parkCenter) {
                        robot.strafe(30, 0.7, 0.0);
                    } else {
                        robot.strafe(-18, 0.7, 0.0);
                    }
                    robot.drive(-16, 0.7, 0, false);

                } else if ((Globals.TEAM_PROP_LOCATION == TeamPropLocation.CENTER) || (Globals.TEAM_PROP_LOCATION == TeamPropLocation.UNKNOWN)) {
                    //  ####
                    //  ####   REAR :  CENTER SPIKE   ########################################################################
                    //  ####
                    robot.drive(23, 0.45, 0.0, false);
                    arm.setLiftSetpoint(10);
                    arm.setExtendSetpoint(7);
                    arm.waitTillArmInPosition();
                    arm.dropPurplePixel();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);

                    if (autoConfig.autoOptions.doubleYellow) {
                        robot.drive(-20, 0.7, 0.1, false);
                        arm.waitTillArmInPosition();  // should already be there.
                        robot.turnTo(-90, 0.5, 0.25);
                        //arm.wristToHome();
                        arm.dropPurplePixel();
                        arm.setLiftSetpoint(Manipulator.LIFT_HOME_ANGLE);
                        robot.drive(27, 0.5, 0.0, false);
                        robot.drive(6, 0.25, 0.0, true);   // Grab Pixel
                        arm.runArmControl(0.5);
                        arm.gotoSafeDriving();
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.drive(-57, 0.7, 0.0, false);
                        robot.strafe(26, 0.7, 0.0);
                        arm.gotoFrontScore();
                        robot.turnTo(90, 0.5, 0);

                    } else {
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.turnTo(90, 0.5, 0);
                        robot.drive(24, 0.7, 0, false);
                        arm.gotoFrontScore();
                        robot.strafe(-6, 0.7, 0.0);
                    }

                    robot.driveToBackdropTag(2);
                    arm.waitTillArmInPosition();
                    arm.dropPixels();
                    arm.runArmControl(1);
                    robot.drive(-4, 0.7, 0, false);
                    arm.gotoHome();
                    robot.turnTo(-90,0.5,0);
                    if (autoConfig.autoOptions.parkCenter) {
                        robot.strafe(24, 0.7, 0.0);
                    } else {
                        robot.strafe(-24, 0.7, 0.0);
                    }
                    robot.drive(-16, 0.5, 0, false);

                } else if (((Globals.ALLIANCE_COLOR == AllianceColor.BLUE) && (Globals.TEAM_PROP_LOCATION == TeamPropLocation.RIGHT_SIDE)) ||
                        ((Globals.ALLIANCE_COLOR == AllianceColor.RED)  && (Globals.TEAM_PROP_LOCATION == TeamPropLocation.LEFT_SIDE))) {
                    //  ####
                    //  ####   REAR :  SPIKE NEAR TRUSS   ########################################################################
                    //  ####
                    robot.drive(18, 0.45, 0.0, false);
                    arm.setLiftSetpoint(10);
                    arm.setExtendSetpoint(5);
                    robot.turnTo(-45, 0.35, 0);
                    arm.waitTillArmInPosition();
                    arm.dropPurplePixel();
                    arm.runArmControl(0.5);
                    arm.setExtendSetpoint(0);
                    robot.turnTo(0, 0.35, 0);

                    if (autoConfig.autoOptions.doubleYellow) {
                        robot.drive(-15, 0.7, 0.1, false);
                        arm.waitTillArmInPosition();  // should already be there.
                        robot.turnTo(-90, 0.5, 0.25);
                        //arm.wristToHome();
                        arm.dropPurplePixel();
                        arm.setLiftSetpoint(Manipulator.LIFT_HOME_ANGLE);
                        robot.drive(27, 0.5, 0.0, false);
                        robot.drive(6, 0.25, 0.0, true);   // Grab Pixel
                        arm.runArmControl(0.5);
                        arm.gotoSafeDriving();
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.drive(-57, 0.7, 0.0, false);
                        robot.strafe(28, 0.7, 0.0);
                        arm.gotoFrontScore();
                        robot.turnTo(90, 0.5, 0);
                    } else {
                        delay(autoConfig.autoOptions.delayYellow);
                        robot.strafe(24, 0.7, 0.0);
                        robot.turnTo(90, 0.5, 0);
                        arm.gotoFrontScore();
                        robot.strafe(-18, 0.7, 0.0);
                    }

                    robot.driveToBackdropTag(3);
                    arm.waitTillArmInPosition();
                    arm.dropPixels();
                    arm.runArmControl(1);

                    robot.drive(-4, 0.7, 0, false);
                    arm.gotoHome();
                    robot.turnTo(-90,0.5,0);
                    if (autoConfig.autoOptions.parkCenter) {
                        robot.strafe(18, 0.7, 0.0);
                    } else {
                        robot.strafe(-30, 0.7, 0.0);
                    }
                    robot.drive(-16, 0.5, 0, false);
                }

            }
        }

        vision.disableAll();
        alert.setState(AlertState.OFF);
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
