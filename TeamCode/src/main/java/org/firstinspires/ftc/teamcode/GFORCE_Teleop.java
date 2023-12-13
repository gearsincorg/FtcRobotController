/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Alert;
import org.firstinspires.ftc.teamcode.subsystems.AlertState;
import org.firstinspires.ftc.teamcode.subsystems.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.subsystems.ManipulatorWristState;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Target;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * This is the Teleop OpMode for G-FORCE's Center Stage robot
 */

@TeleOp(name="G-FORCE TELEOP", group = "AAA")
public class GFORCE_Teleop extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   =  0.7 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  =  0.7 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     =  0.6 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double GYRO_ADJUST_TC     =  0.5 ; // time constant used to slew heading to apriltag derived value
    final double LAUNCHER_SPEED = 1200 ; //

    private ElapsedTime rumbleTime   = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    // Driving parameters
    boolean autoHeading = false;
    boolean hasRumbled  = false;

    boolean thisGyroReset = false;
    boolean lastGyroReset = false;
    double gyroHeadingReset = 0;

    Target stackTarget = new Target();

    // get an instance of the "Drive" class.
    Robot       robot   = Robot.getInstance();
    Manipulator arm     = new Manipulator(this);
    Drone       drone   = new Drone(this);
    Vision      vision  = new Vision(this);
    Alert       alert   = new Alert(this);
    AutoConfig  autoConfig  = new AutoConfig(this);

    Gamepad.RumbleEffect customRumbleEffectD1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffectD2;    // Use to build a custom rumble sequence.

    @Override public void runOpMode()
    {
        Globals.IS_AUTO = false;

        // Initialize the robot's hardware & Turn on telemetry
        robot.initialize(this, arm, vision,true);
        arm.initialize(true);
        vision.initialize(true);
        drone.initialize(true);
        alert.initialize(false);
        autoConfig.initialize();

        if (autoConfig.autoOptions.redAlliance )
            Globals.ALLIANCE_COLOR = AllianceColor.RED;
        else
            Globals.ALLIANCE_COLOR = AllianceColor.BLUE;

        if (!Globals.ARM_HAS_HOMED) {
            arm.homeArm();
        }

        alert.setState(AlertState.TELEOP_GRABBER);

        // vision.enableAprilTag();
        robot.resetOdometry();

        // reset wrist if in unknown state
        if (Globals.WRIST_STATE == ManipulatorWristState.UNKNOWN) {
            arm.wristToHome();
        }

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to drive");
        telemetry.update();

        // Example 1. a)   start by creating a three-pulse rumble sequence: right, LEFT, LEFT
        customRumbleEffectD1 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .build();

        // Example 1. a)   start by creating a three-pulse rumble sequence: right, LEFT, LEFT
        customRumbleEffectD2 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 1500) //  Pause for 1.5 mSec
                .build();

        // Display Sensors while waiting...
        while (opModeInInit()) {
            robot.readSensors();
            arm.readSensors();
            arm.runManualGrippers();
            alert.update();

            telemetry.update();
        }

        // prep for Teleop.
        arm.setLiftSetpoint(0);
        rumbleTime.reset();
        robot.yawController.reset();

        arm.setRangeEnable(true);

        while (opModeIsActive())
        {
            robot.readSensors();
            arm.runArmControl();
            arm.manualArmControl();
            alert.update();

            // Attempt to reset the gyro if robot is under the truss and sees the small apriltag within 5 degrees.
            // Enable with driver right joystick click.
            if (gamepad1.share) {
                stackTarget = vision.findStack();
                if ( stackTarget.targetFound &&
                    (stackTarget.rangeInch > 50) && (stackTarget.rangeInch < 58) &&
                    (Math.abs(Math.abs(robot.heading) - 90) < 15) &&
                    (Math.abs(stackTarget.bearingDeg) < 15) )  {
                    // reset the heading assuming that the target is actually on the +/- 90 axis)
                    // Apply the new heading with a filter
                    double headingError = -stackTarget.bearingDeg ;
                    double filtHeading = robot.heading + (headingError * GYRO_ADJUST_TC);
                    telemetry.addData("GYRO DRIFT", "H:E:F %4.1f : %4.2f : %4.1f", robot.heading, headingError, filtHeading);
                    robot.correctHeading(robot.normalizeHeading(filtHeading));
                 }
            }

            // check for rumble
            if (!hasRumbled && (rumbleTime.seconds() > 80))  {
                telemetry.addLine("=== RUMBLE ===");
                gamepad1.runRumbleEffect(customRumbleEffectD1);
                gamepad2.runRumbleEffect(customRumbleEffectD2);
                hasRumbled = true;
            }

            //  ==  CoPilot Controls  ===================================

            //controls for Automatic Arm movements
            if (gamepad2.a){
                arm.setRangeEnable(true);
                arm.gotoHome();
            } else if (gamepad2.x){
                arm.setRangeEnable(false);
                arm.gotoSafeDriving();
            } else if (gamepad2.b) {
                arm.setRangeEnable(false);
                arm.gotoFrontScore();
            } else if (gamepad2.y) {
                arm.setRangeEnable(false);
                arm.gotoBackScore();
            }

            //controls for our drone laucher
            if (gamepad2.dpad_up){
                drone.setDroneSpeed(LAUNCHER_SPEED);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                drone.fireDrone();
            } else if (gamepad2.dpad_down) {
                drone.stopLauncher();
            }

            //  switch to Power lifing mode when
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                arm.gotoHang();
            }

            if (arm.readyToHang()) {
                alert.setState(AlertState.READY_TO_LIFT);
                if ((gamepad2.left_trigger > 0.5) && (gamepad2.right_trigger > 0.5) && (arm.liftAngle > 0.0)) {
                    arm.setExtendSetpoint(Manipulator.EXTEND_LIFT_LENGTH - 0.5);
                    arm.powerLift();
                } else {
                    arm.setExtendSetpoint(Manipulator.EXTEND_LIFT_LENGTH);
                    arm.powerHold();
                }
            }

            //  ==  Pilot Controls  =======================================

            // Allow driver to reset the gyro
            // Start and stop reser using the touchpad button.
            // remember any button presses during the reset and use this
            // as the target for the reset when the touchpad is released.

            if (gamepad1.touchpad){
                // set heading
                if (gamepad1.y) {
                   robot.setHeading(0);
                } else if (gamepad1.b) {
                    robot.setHeading(-90);
                } else if (gamepad1.a) {
                    robot.setHeading(180);
                } else if (gamepad1.x) {
                    robot.setHeading(90);
                }
            } else {
                // set heading set-point
                if (gamepad1.triangle){
                    robot.yawController.reset(0);
                } else if (gamepad1.square){
                    robot.yawController.reset(90);
                } else if(gamepad1.cross){
                    robot.yawController.reset(180);
                } else if(gamepad1.circle){
                    robot.yawController.reset(270);
                }
            }

            lastGyroReset = thisGyroReset;

            if (gamepad1.left_trigger > 0.25) {
                arm.openLeftGrabber();
            } else if (gamepad1.left_bumper || arm.pixelLeftInRange) {
                arm.closeLeftGrabber();
            }

            if (gamepad1.right_trigger > 0.25) {
                arm.openRightGrabber();
            } else if (gamepad1.right_bumper  || arm.pixelRightInRange){
                arm.closeRightGrabber();
            }

            // arm.autoPickup();

            // read joystick values and scale according to limits in Robot class
            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;

            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED * 0.8;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED * 0.8;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED * 0.8;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED * 0.8;
            }



            // Apply field centric driving
            double fieldAxial  = (drive * Math.cos(Math.toRadians(-robot.getHeading()))) -
                    (strafe * Math.sin(Math.toRadians(-robot.getHeading())));

            double fieldLateral = (drive * Math.sin(Math.toRadians(-robot.getHeading()))) +
                    (strafe * Math.cos(Math.toRadians(-robot.getHeading())));

            drive = fieldAxial;
            strafe = fieldLateral;

            // Is the driver turning the robot, or should it hold its heading?
            if (Math.abs(yaw) > 0.05) {
                // driver is commanding robot to turn, so turn off auto heading.
                autoHeading = false;
            } else {
                // If we are not already locked, wait for robot to stop rotating (<2 deg per second) and then lock-in the current heading.
                if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                    robot.yawController.reset(robot.getHeading());
                    autoHeading = true;
                }
            }

            // If auto heading is on, override manual yaw with the value generated by the heading controller.
            if (autoHeading) {
                yaw = robot.yawController.getOutput(robot.getHeading());
                telemetry.addData("Heading Error", "%6.3f", robot.yawController.getSetpoint() - robot.heading);
            }

            // Drive the wheels based on the desired axis motions
            // Prevent wheels turning if robot is tilted (eg: lifting)
            if (Math.abs(robot.pitch) < 20 ) {
                robot.moveRobot(drive, strafe, yaw);
            } else {
                robot.stopRobot();
            }
        }

        Globals.ARM_HAS_HOMED = false;
        Globals.WRIST_STATE = ManipulatorWristState.UNKNOWN;
        arm.openGrabbers();
    }
}
