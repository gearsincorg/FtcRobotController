/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AutoConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drone;
import org.firstinspires.ftc.teamcode.subsystems.Globals;
import org.firstinspires.ftc.teamcode.subsystems.Manipulator;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Side;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.WristState;

/*
 * This OpMode used the IMU gyro to stabilize the heading when the operator is nor requesting a turn.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 */


@TeleOp(name="G-FORCE Teleop", group = "AAA")
public class GFORCE_Teleop extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   =  0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  =  0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     =  0.4 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double HEADING_HOLD_TIME  =  5.0 ; // How long to hold heading once all driver input stops. (Avoids effects of Gyro Drift)

    private ElapsedTime stopTime   = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    // Driving parameters
    boolean autoHeading = false;
    boolean liftingAction = false;
    boolean lastLiftingAction = false;
    double  noDriftHeading = 0;

    // get an instance of the "Drive" class.
    Robot robot = Robot.getInstance();
    Manipulator arm = new Manipulator(this);
    Drone drone = new Drone(this);
    Vision vision = new Vision(this);
    AutoConfig autoConfig  = new AutoConfig(this);


    @Override public void runOpMode()
    {
        Globals.IS_AUTO = false;

        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(this, arm, vision,true);
        arm.initialize(true);
        drone.initialize(true);
        robot.resetOdometry();

        vision.initialize(true);
        vision.enableAprilTag();

        autoConfig.initialize();
        if (autoConfig.autoOptions.redAlliance )
            Globals.ALLIANCE = Side.RED;
        else
            Globals.ALLIANCE = Side.BLUE;

        if (!Globals.ARM_HAS_HOMED) {
            arm.homeArm();
        }

        // reset wrist if in unknown state
        if (Globals.WRIST_STATE == WristState.UNKNOWN) {
            arm.wristToHome();
        }

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to drive");
        telemetry.update();
        while (opModeInInit()) {
            arm.runManualGrippers();
            robot.readSensors();
            arm.readSensors();
            telemetry.update();
        }

        arm.setLiftSetpoint(0);

        // Reset heading control loop to lock in current heading
        robot.yawController.reset();

        arm.setRangeEnable(true);

        while (opModeIsActive())
        {
            robot.readSensors();
            arm.runArmControl();
            arm.manualArmControl();

            vision.telemetryAprilTag();

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
                drone.runLauncher();
            } else if (gamepad2.dpad_right || gamepad2.dpad_right) {
                drone.fireDrone();
            } else if (gamepad2.dpad_down) {
                drone.stopLauncher();
            }

            //  switch to lifing mode when
            liftingAction = gamepad2.left_stick_button && gamepad2.right_stick_button;
            if (!lastLiftingAction && liftingAction) {
                arm.enablePowerLifting();
            }

            if (arm.weArePowerLifting()) {
                if ((gamepad2.left_trigger > 0.5) && (gamepad2.right_trigger > 0.5) && (arm.liftAngle > 0.0)) {
                    arm.powerLift();
                } else {
                    arm.powerHold();
                }
            }

            //  ==  Pilot Controls  =======================================

            // Allow driver to reset the gyro
            if (gamepad1.touchpad){
                robot.resetHeading();
                robot.resetOdometry();
            }

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
                strafe = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED / 2.0;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED / 2.0;
            }

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
        Globals.WRIST_STATE = WristState.UNKNOWN;
        arm.openGrabbers();
    }
}
