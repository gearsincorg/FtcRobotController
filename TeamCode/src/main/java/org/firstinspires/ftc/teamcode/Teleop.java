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

import static com.pedropathing.math.MathFunctions.normalizeAngleSigned;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "G-FORCE TELEOP", group = "Sensor")
public class Teleop extends OpMode {

    public static Pose startingPose; //See ExampleAuto to understand how to use this

    private Follower            follower;
    private boolean             automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager    telemetryM;
    private boolean             slowMode = false;

    private boolean             headingLocked = false;
    private double              headingSetpoint = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(36, -12))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        //Lazy curve generation might look a little different to how we usually make paths,
        //This is because we are using a Lambda expression instead of calling follower.pathbuilder()
        //To use this though, we declare this as a Supplier<PathChain>. And we to call the path chain we instead do pathChain.get()
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        // mode controls =================================

        // Home the pose (location and heading)
        if (gamepad1.touchpadWasPressed()){
            follower.setPose(new Pose());
            headingSetpoint = 0.0;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.aWasPressed() || !follower.isBusy())) {
            headingSetpoint = follower.getHeading();
            follower.startTeleopDrive();
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

            // Lock heading if we aren't trying to turn.
            if (yaw == 0 ) {
                if (headingLocked) {
                    yaw = normalizeAngleSigned(headingSetpoint - follower.getHeading()) * HEADING_GAIN;
                } else if (Math.abs(follower.getAngularVelocity()) < MIN_ROTATE) {
                    headingSetpoint = follower.getHeading();
                    headingLocked = true;
                }
            } else {
                headingLocked = false;
            }

            follower.setTeleOpDrive(axial, lateral, yaw, false);
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}