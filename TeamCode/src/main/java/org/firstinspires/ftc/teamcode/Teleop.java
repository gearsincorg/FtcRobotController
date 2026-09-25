/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.Supplier;

@TeleOp(name = "G-FORCE TELEOP", group = "Sensor")
public class Teleop extends OpMode {

    public static Pose startingPose; //See ExampleAuto to understand how to use this

    private Follower            follower;
    private boolean             automatedDrive;
    private Supplier<Path>      path;
    private boolean             slowMode = false;

    private boolean             headingLocked = false;
    private double              headingSetpoint = 0;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();

        // mode controls =================================

        // Home the pose (location and heading)
        if (gamepad1.touchpadWasPressed()){
            follower.setPose(Pose.zero());
            headingSetpoint = 0.0;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Automated PathFollowing (A starts it, and A again cancels it)
        boolean aPressed = gamepad1.aWasPressed();
        if (aPressed && !automatedDrive) {
            follower.follow(path.get());
            automatedDrive = true;
        } else if (automatedDrive && (aPressed || !follower.isBusy())) {
            //Stop automated following if cancelled or if the follower is done
            follower.stop();
            headingSetpoint = follower.pose().heading();
            headingLocked = true;
            automatedDrive = false;
        }

        // Manual driving ===================================
        final double SLOW_MULTIPLIER = 0.5;
        final double HEADING_GAIN    = 1.0;
        final double MIN_ROTATE      = 0.1;

        if (!automatedDrive) {
            double axial    = -gamepad1.left_stick_y * (slowMode ? SLOW_MULTIPLIER : 1.0);
            double lateral  = -gamepad1.left_stick_x * (slowMode ? SLOW_MULTIPLIER : 1.0);
            double yaw      = -gamepad1.right_stick_x * (slowMode ? SLOW_MULTIPLIER : 1.0);

            double heading = follower.pose().heading();

            // Lock heading if we aren't trying to turn.
            if (yaw == 0 ) {
                if (headingLocked) {
                    yaw = Angle.normalizeSigned(headingSetpoint - heading) * HEADING_GAIN;
                } else if (Math.abs(follower.velocity().omega) < MIN_ROTATE) {
                    headingSetpoint = heading;
                    headingLocked = true;
                }
            } else {
                headingLocked = false;
            }

            // Robot-centric drive
            follower.manual(axial, lateral, yaw);
        }

        telemetry.addData("position", follower.pose());
        telemetry.addData("velocity", follower.velocity());
        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.update();
    }
}
