/* Copyright (c) 2019 G-FORCE.
 *
 * This OpMode is the G-FORCE SKYSTONE Teleop opmode
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="G-FORCE Teleop", group="!Competition")
public class GFORCE_TeleOp extends LinearOpMode {

    public final double SLOW_AXIAL_JS_SCALE = 0.2;
    public final double NORMAL_AXIAL_JS_SCALE = 1.0;

    public final double SLOW_YAW_JS_SCALE = 0.15;
    public final double NORMAL_YAW_JS_SCALE = 0.25;

    public final int shooterSpeed = 2700;

    private ElapsedTime neutralTime = new ElapsedTime();

    /* Declare OpMode members. */
    GFORCE_Hardware robot = new GFORCE_Hardware();

    @Override
    public void runOpMode() {
        double forwardBack;
        double rotate;

        double axialVel;
        double yawVel;

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
        robot.startMotion();

        // Run until the end of the match (Driver presses STOP)
        while (opModeIsActive()) {
            robot.updateMotion();  // Read all sensors and calculate motions

            //Driver Controls
            if (gamepad1.back && gamepad1.start) {
                robot.resetHeading();
                desiredHeading = 0;
            }

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            forwardBack = -gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;

            if (gamepad1.left_trigger > 0.5) {
                forwardBack *= SLOW_AXIAL_JS_SCALE;
                rotate *= SLOW_YAW_JS_SCALE;
            } else {
                forwardBack *= NORMAL_AXIAL_JS_SCALE;
                rotate *= NORMAL_YAW_JS_SCALE;
            }

            //Scale velocities to mm per second
            axialVel = forwardBack * robot.MAX_VELOCITY_MMPS;
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

            // Disable correction if JS are neutral for more than 2 seconds
            neutralSticks = ((forwardBack == 0) && (rotate == 0));
            if (!neutralSticks)
                neutralTime.reset();

            if (autoHeadingOn && (neutralTime.time() < 3)) {
                robot.setYawVelocityToHoldHeading(desiredHeading);
            } else {
                robot.setYawVelocity(yawVel);
                desiredHeading = robot.currentHeading;
            }

            robot.setAxialVelocity(axialVel);
            robot.moveRobotVelocity();

            if (gamepad1.right_bumper) {
                robot.leftShooter.setVelocity(shooterSpeed);
                robot.rightShooter.setVelocity(shooterSpeed);
            } else {
                robot.leftShooter.setVelocity(0);
                robot.rightShooter.setVelocity(0);
            }

            // Send telemetry message to signify robot running
            robot.showEncoders();
        }

    }
}
