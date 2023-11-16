/* Copyright (c) 2023 Phil Malone. All rights reserved. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates a teleop OpMode for an Omni bot.
 * This OpMode used the IMU gyro to stabilize the heading when the operator is nor requesting a turn.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 */


@TeleOp(name="G-FORCE Teleop", group = "AAA")
public class GFORCE_Teleop extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   =  0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  =  0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     =  0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double HEADING_HOLD_TIME  = 10.0 ; // How long to hold heading once all driver input stops. (Avoids effects of Gyro Drift)

    private ElapsedTime stopTime   = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    // Driving parameters
    boolean autoHeading = false;

    // get an instance of the "Drive" class.
    Robot robot = new Robot(this);
    Manipulator arm = new Manipulator(this);
    Drone drone = new Drone(this);

    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true);
        arm.initialize(true);
        drone.initialize(true);
        arm.homeArm();
        robot.resetOdometry();

        // Wait for driver to press start
        telemetry.addData(">", "Touch Play to drive");
        telemetry.update();
        waitForStart();

        arm.setLiftSetpoint(0);

        // Reset heading control loop to lock in current heading
        robot.yawController.reset();

        while (opModeIsActive())
        {
            robot.readSensors();
            arm.readSensors();
            arm.manualArmControl();
            arm.runLiftControl();
            arm.runExtendControl();
            arm.runStateMachine();

            //  ==  CoPilot Controls  ===================================

            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                arm.liftingIsActive = true;
            }

            if (arm.liftingIsActive) {
                if ((gamepad2.left_trigger > 0.5) && (gamepad2.right_trigger > 0.5)) {

                }
            }

            /*
            // Manually lidt and lower the arm
            if (gamepad2.y) {
                arm.setLiftSetpoint(Manipulator.LIFT_BACK_ANGLE);
            } else if (gamepad2.a) {
                arm.setLiftSetpoint(Manipulator.LIFT_HOME_ANGLE);
            } else if (gamepad2.b) {
                arm.setLiftSetpoint(Manipulator.EXTEND_FRONT_DISTANCE);
            }  else if (gamepad2.x) {
                arm.setLiftSetpoint(Manipulator.LIFT_HOVER_ANGLE);
            }

            // Manually extend and retract the arm
            if (gamepad2.dpad_down) {
                arm.setExtendSetpoint(Manipulator.EXTEND_HOME_DISTANCE);
            } else if (gamepad2.dpad_left) {
                arm.setExtendSetpoint(Manipulator.EXTEND_FRONT_DISTANCE);
            } else if (gamepad2.dpad_right) {
                arm.setExtendSetpoint(12);
            }  else if (gamepad2.dpad_up) {
                arm.setExtendSetpoint(18);
            }
            */

            //  ==  Pilot Controls  =======================================

            // Allow driver to reset the gyro
            if (gamepad1.options && gamepad1.share){
                robot.resetHeading();
                robot.resetOdometry();
            }

            //controls for our drone laucher
            if (gamepad1.dpad_up){
                drone.setTiltAngle(1);
                drone.runLauncher();
            } else if (gamepad1.dpad_right) {
                drone.fireDrone();
            } else if (gamepad1.dpad_down) {
                drone.setTiltAngle(0);
                drone.stopLauncher();
            }

            //controls for Automatic pilot movements
            if (gamepad1.a){
                arm.gotoHome();
            } else if (gamepad1.x){
                arm.gotoSafeDriving();
            } else if (gamepad1.b) {
                arm.gotoFrontScore();
            } else if (gamepad1.y) {
                arm.gotoBackScore();
            }

            if (gamepad1.left_trigger > 0.25) {
                arm.openLeftGrabber();
            } else if (gamepad1.left_bumper || arm.pixelLeftInRange) {
                arm.closeLeftGrabber();
                // arm.autoPickup();
            }

            if (gamepad1.right_trigger > 0.25) {
                arm.openRightGrabber();
            } else if (gamepad1.right_bumper  || arm.pixelRightInRange){
                arm.closeRightGrabber();
            }

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

            // Keep track of how long the controls have been idle...
            // If the robot has just been sitting here for a while, make heading setpoint track any gyro drift to prevent rotating.
            if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());  // just keep tracking the current heading
                }
            } else {
                stopTime.reset();
            }

            // If auto heading is on, override manual yaw with the value generated by the heading controller.
            if (autoHeading) {
                yaw = robot.yawController.getOutput(robot.getHeading());
                telemetry.addData("Heading Error", "%6.3f", robot.yawController.getSetpoint() - robot.heading);
            }

            //  Drive the wheels based on the desired axis motions
            robot.moveRobot(drive, strafe, yaw);
        }
    }
}
