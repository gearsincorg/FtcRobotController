/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Teleop opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="G-FORCE Teleop", group="!Competition")
public class Teleoperation extends LinearOpMode {

    public final double SLOW_AXIAL_JS_SCALE = 0.2;
    public final double NORMAL_AXIAL_JS_SCALE = 0.6;

    public final double SLOW_LATERAL_JS_SCALE = 0.2;
    public final double NORMAL_LATERAL_JS_SCALE = 0.6;

    public final double SLOW_YAW_JS_SCALE = 0.15;
    public final double NORMAL_YAW_JS_SCALE = 0.25;

    private ElapsedTime neutralTime = new ElapsedTime();

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        double forwardBack;
        double rotate;
        double rightLeft;

        double axialVel;
        double yawVel;
        double lateralVel;

        double desiredHeading = 0;
        boolean neutralSticks = true;
        boolean autoHeadingOn = false;

        /* Initialize the hardware variables.
         * The init() method of the Hardware class does all the work here
         */
        robot.init(this);

        // Wait for the game to start (Driver presses PLAY)
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();

        waitForStart();

        // Run until the end of the match (Driver presses STOP)
        while (opModeIsActive()) {

            //Driver Controls
            if (gamepad1.back && gamepad1.start) {
                robot.resetHeading();
                desiredHeading = 0;
            }

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            forwardBack = -gamepad1.left_stick_y;
            rightLeft = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;

            //Three different speeds: slow, medium, and fast depending on what trigger the driver is holding

            if (gamepad1.left_trigger > 0.5) {
                forwardBack *= SLOW_AXIAL_JS_SCALE;
                rightLeft *= SLOW_LATERAL_JS_SCALE;
                rotate *= SLOW_YAW_JS_SCALE;
            } else {
                forwardBack *= NORMAL_AXIAL_JS_SCALE;
                rightLeft *= NORMAL_LATERAL_JS_SCALE;
                rotate *= NORMAL_YAW_JS_SCALE;
            }


            //Field Centric Motion
            axialVel = -((forwardBack * Math.sin(Math.toRadians(robot.currentHeading))) +
                    (rightLeft * Math.cos(Math.toRadians(robot.currentHeading))));

            lateralVel = (forwardBack * Math.cos(Math.toRadians(robot.currentHeading))) -
                    (rightLeft * Math.sin(Math.toRadians(robot.currentHeading)));

            //Scale velocities to mm per second
            axialVel *= robot.MAX_VELOCITY_MMPS;
            lateralVel *= robot.MAX_VELOCITY_MMPS;
            yawVel = rotate * robot.MAX_VELOCITY_MMPS;

            // Control Yaw, using manual or auto correction
            if (rotate != 0) {
                // We are turning with the joystick
                autoHeadingOn = false;
            } else if (!autoHeadingOn && robot.notTurning()) {
                // We have just stopped turning, so lock in current heading
                desiredHeading = robot.currentHeading;
                autoHeadingOn = true;
            }

            robot.setAxialVelocity(axialVel);
            robot.setLateralVelocity(lateralVel);

            // Disable correction if JS are neutral for more than 2 seconds
            neutralSticks = ((forwardBack == 0) && (rightLeft == 0) &&  (rotate == 0));
            if (!neutralSticks)
                neutralTime.reset();

            if (autoHeadingOn && (neutralTime.time() < 3)) {
                robot.setYawVelocityToHoldHeading(desiredHeading);
            } else {
                robot.setYawVelocity(yawVel);
                desiredHeading = robot.currentHeading;
            }

            robot.moveRobotVelocity();


            // Send telemetry message to signify robot running
            robot.showEncoders();
        }

    }
}
