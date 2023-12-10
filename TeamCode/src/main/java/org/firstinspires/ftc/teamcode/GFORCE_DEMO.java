/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * This OpMode used the IMU gyro to stabilize the heading when the operator is nor requesting a turn.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 */


@TeleOp(name="G-FORCE DEMO - no drive", group = "Demo")
public class GFORCE_DEMO extends LinearOpMode
{
    final double LAUNCHER_SPEED = 1200 ; //

    private ElapsedTime rumbleTime   = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    // get an instance of the "Drive" class.
    Robot       robot   = Robot.getInstance();
    Manipulator arm     = new Manipulator(this);
    Drone       drone   = new Drone(this);
    Vision      vision  = new Vision(this);
    Alert       alert   = new Alert(this);
    AutoConfig  autoConfig  = new AutoConfig(this);

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

        vision.enableAprilTag();
        robot.resetOdometry();

        // reset wrist if in unknown state
        if (Globals.WRIST_STATE == ManipulatorWristState.UNKNOWN) {
            arm.wristToHome();
        }

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to activate Demo");
        telemetry.update();

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
        arm.setRangeEnable(true);

        while (opModeIsActive()) {
            robot.readSensors();
            arm.runArmControl();
            arm.manualArmControl();
            alert.update();

            //  ==  CoPilot Controls  ===================================

            //controls for Automatic Arm movements
            if (gamepad2.a) {
                arm.setRangeEnable(true);
                arm.gotoHome();
            } else if (gamepad2.x) {
                arm.setRangeEnable(false);
                arm.gotoSafeDriving();
            } else if (gamepad2.b) {
                arm.setRangeEnable(true);
                arm.gotoFrontScore();
            } else if (gamepad2.y) {
                arm.setRangeEnable(false);
                arm.gotoBackScore();
            }

            //controls for our drone laucher
            if (gamepad2.dpad_up) {
                drone.setDroneSpeed(LAUNCHER_SPEED);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                drone.fireDrone();
            } else if (gamepad2.dpad_down) {
                drone.stopLauncher();
            }

            //  ==  Assign grabbers to copilot for demo  =======================================
            if (gamepad2.left_trigger > 0.25) {
                arm.openLeftGrabber();
            } else if (gamepad2.left_bumper || arm.pixelLeftInRange) {
                arm.closeLeftGrabber();
            }

            if (gamepad2.right_trigger > 0.25) {
                arm.openRightGrabber();
            } else if (gamepad2.right_bumper || arm.pixelRightInRange) {
                arm.closeRightGrabber();
            }
        }

        Globals.ARM_HAS_HOMED = false;
        Globals.WRIST_STATE = ManipulatorWristState.UNKNOWN;
        arm.openGrabbers();
    }
}
