/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Collector Test", group="Linear Opmode")
//@Disabled
public class CollectorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontCollector = null;
    private DcMotorEx midCollector = null;
    private RevTouchSensor midCollectorDown = null;

    final double MOTOR_INCREASE = 100;
    final double MAX_RPM = 2700;

    double frontSpeed = MAX_RPM;
    double midSpeed = MAX_RPM;

    public boolean frontFast = false;
    public boolean frontSlow = false;
    public boolean midFast = false;
    public boolean midSlow = false;

    public boolean lastFrontFast = false;
    public boolean lastFrontSlow = false;
    public boolean lastMidFast = false;
    public boolean lastMidSlow = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontCollector = hardwareMap.get(DcMotorEx.class, "m1");
        midCollector = hardwareMap.get(DcMotorEx.class, "m2");
        midCollectorDown = hardwareMap.get(RevTouchSensor.class,"midTouch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontCollector.setDirection(DcMotorEx.Direction.REVERSE);
        midCollector.setDirection(DcMotorEx.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontVelocity = frontCollector.getVelocity();
            double midVelocity = midCollector.getVelocity();

            frontFast = gamepad1.left_bumper;
            frontSlow = (gamepad1.left_trigger > 0.5);
            midFast = gamepad1.right_bumper;
            midSlow = (gamepad1.right_trigger > 0.5);

            // Look for button clicks and adjust speed
            if (frontFast && !lastFrontFast) {
                frontSpeed += MOTOR_INCREASE;
            } else if (frontSlow && !lastFrontSlow) {
                frontSpeed -= MOTOR_INCREASE;
            }

            if (midFast && !lastMidFast) {
                midSpeed += MOTOR_INCREASE;
            } else if (midSlow && !lastMidSlow) {
                midSpeed -= MOTOR_INCREASE;
            }

            // Clipping so that it does not go too fast
            frontSpeed = Range.clip(frontSpeed,0, MAX_RPM);
            midSpeed = Range.clip(midSpeed, 0, MAX_RPM);

            // When y pressed run front collector
            if (gamepad1.y) {
                if (midCollectorDown.isPressed()) {
                    frontCollector.setVelocity(frontSpeed);
                } else {
                    frontCollector.setVelocity(-frontSpeed/4);
                }
            } else {
                frontCollector.setVelocity(0);
            }

            //When a pressed run mid collector
            if (gamepad1.a) {
                midCollector.setVelocity(midSpeed);
            } else {
                midCollector.setVelocity(0);
            }


            // Set to previous values before looping again
            lastFrontFast = frontFast;
            lastFrontSlow = frontSlow;
            lastMidFast = midFast;
            lastMidSlow = midSlow;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData(":", "Left bumper increase front (0)");
            telemetry.addData(":", "Right bumper increase mid (1)");
            telemetry.addData("Measured Velocities", " front (%.2f), mid (%.2f)", frontVelocity, midVelocity);
            telemetry.addData("Set Velocities", " front (%.2f), mid (%.2f)", frontSpeed, midSpeed);
            telemetry.update();
        }
    }
}
