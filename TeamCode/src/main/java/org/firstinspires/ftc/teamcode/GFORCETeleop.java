/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot using Essential Mecanum functions.
 * An external "EssentialMecanumRobot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="GFORCE Teleop", group = "Mr. Phil")
public class GFORCETeleop extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED   = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_STRAFE_SPEED  = 0.8 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double SAFE_YAW_SPEED     = 0.5 ; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double HEADING_HOLD_TIME  = 10.0 ; // How long (in seconds) to hold heading once all driver input stops. (This Avoids effects of Gyro Drift)
    final double IN_FRONT_ANGLE = 0.14 ;
    final double VERY_IN_FRONT_ANGLE = 0.05 ;
    final double CLICK_ON_SPEED = 0.2 ;
    final double APPROACH_SPEED = 0.3 ;
    final double STRAFE_GAIN = 1.5 ;


    // local parameters
    ElapsedTime stopTime   = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading    = false; // used to indicate when heading should be locked.

    // get an instance of the "Robot" class.
    DriveSubsystem robot = new DriveSubsystem(this);
    ArmSubsystem arm = new ArmSubsystem(this);
    VisionSubsystem camera = new VisionSubsystem(this);

    @Override public void runOpMode()
    {
        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true);
        camera.initilaize(true);
        arm.initialize(true);


        // Wait for driver to press start
        while(opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            robot.readSensors();
            arm.runArmControl();
            telemetry.update();
        }

        arm.resetEncoders();
        arm.setSetpointInches(arm.SPECIMIN_HEIGHT);

        while (opModeIsActive())
        {
            // Get the latest sensor data every time around the loop.
            robot.readSensors();
            arm.runArmControl();


            // read joystick values and scale according to limits set at top of this file
            double drive  = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;      //  Fwd/back on left stick
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED;     //  Left/Right on left stick
            double yaw    = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       //  Rotate on right stick

            //  OR... For special conditions, Use the DPAD to make slow-mo orthogonal motions.  Adjust the divider to your needs.
            if (gamepad1.dpad_left) {
                strafe = SAFE_DRIVE_SPEED / 4.0;
            } else if (gamepad1.dpad_right) {
                strafe = -SAFE_DRIVE_SPEED / 4.0;
            } else if (gamepad1.dpad_up) {
                drive = SAFE_DRIVE_SPEED / 4.0;
            } else if (gamepad1.dpad_down) {
                drive = -SAFE_STRAFE_SPEED / 4.0;
            }

            if (gamepad1.right_trigger > 0.25) {
                double xError = 0 - camera.getTargetX();
                strafe = xError * STRAFE_GAIN;

                double yError = robot.frontRange - 2;
                if ((robot.frontRange > 5) && (Math.abs(xError) < IN_FRONT_ANGLE)) {
                    drive =APPROACH_SPEED;
                } else if ((robot.frontRange <= 5) && (Math.abs(xError) < VERY_IN_FRONT_ANGLE)) {
                    drive =CLICK_ON_SPEED;
                }

            }

            // This is where we keep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
            // Is the driver turning the robot, or should it hold its heading?
            if (Math.abs(yaw) > 0.05) {
                // driver is commanding robot to turn, so turn off auto heading.
                autoHeading = false;
            } else {
                // If we are not already locked, wait for robot to stop rotating (<2 deg per second) and then lock-in the current heading.
                if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                    robot.yawController.reset(robot.getHeading());  // Lock in the current heading
                    autoHeading = true;
                }
            }

            // If auto heading is on, override manual yaw with the value generated by the heading controller.
            if (autoHeading) {
                yaw = robot.yawController.getOutput(robot.getHeading());
            }

            //  Drive the wheels based on the desired axis motions
            robot.moveRobot(drive, strafe, yaw);

            // If the robot has just been sitting here for a while, make heading setpoint track any gyro drift to prevent rotating.
            if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
                if (stopTime.time() > HEADING_HOLD_TIME) {
                    robot.yawController.reset(robot.getHeading());  // just keep tracking the current heading
                }
            } else {
                stopTime.reset();
            }

            // use the gamepad to set the arms setpoint
            if (gamepad1.triangle) {
                arm.setSetpointInches(arm.HIGH_CHAMBER);
            } else if (gamepad1.cross) {
                arm.homeTheArm();
            } else if (gamepad1.square){
                arm.setSetpointInches(arm.HIGH_CHAMBER_RELEASE);
            }
        }
    }
}