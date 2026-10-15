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

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.FlowerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "G-FORCE TELEOP", group = "Competition")
public class GFORCE_Teleop extends CommandOpMode {

    // Declare Subsystems
    private FlowerSubsystem     flowerSubsystem;
    private VisionSubsystem     visionSubsystem;
    private Follower            driveSubsystem;

    // Point-at-target tuning
    private static final double TARGET_TURN_GAIN = 0.04;
    private static final double MAX_TARGET_TURN  = 0.4;

    // class Methods
    private boolean             headingLocked = false;
    private double              headingSetpoint = 0;

    // == Standard Methods ===================================================
    @Override
    public void initialize() {
        // Create subsystems here (not as field initializers) so they register with the
        // scheduler after CommandOpMode has reset it.  SubsystemBase registers itself.
        flowerSubsystem = new FlowerSubsystem(this);
        visionSubsystem = new VisionSubsystem(this);
        flowerSubsystem.init();
        visionSubsystem.init();

        // Setup robot
        driveSubsystem = Constants.create(hardwareMap);

        // Connect buttons with commands
        bindButtons();
    }

    private void showTelemetry() {
        telemetry.addData("pos X: Y", "%5.1f : %5.1f", driveSubsystem.pose().x(), driveSubsystem.pose().y());
        telemetry.addData("vel X: Y: O", "%5.1f : %5.1f : %5.0f", driveSubsystem.velocity().vx,
                            driveSubsystem.velocity().vy,  driveSubsystem.velocity().omega);
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        manualDrive();
        driveSubsystem.update();
        showTelemetry();
    }

    // == Additional Methods ===================================================

    public void manualDrive() {
        // Manual driving ===================================
        final double SLOW_MULTIPLIER = 0.5;
        final double HEADING_GAIN    = 1.0;
        final double MIN_ROTATE      = 0.1;

        double axial    = -gamepad1.left_stick_y  * SLOW_MULTIPLIER;
        double lateral  = -gamepad1.left_stick_x  * SLOW_MULTIPLIER;
        double yaw      = -gamepad1.right_stick_x * SLOW_MULTIPLIER;

        double heading = driveSubsystem.pose().heading();

        if (gamepad1.right_bumper && visionSubsystem.hasTarget()) {
            // Point at the target cluster: use the bearing (+ is CCW) as the error.
            yaw = Range.clip(visionSubsystem.getTargetPose().bearing * TARGET_TURN_GAIN,
                             -MAX_TARGET_TURN, MAX_TARGET_TURN);
            headingLocked = false;
        } else if (yaw == 0 ) {
            // Lock heading if we aren't trying to turn.
            if (headingLocked) {
                yaw = Angle.normalizeSigned(headingSetpoint - heading) * HEADING_GAIN;
            } else if (Math.abs(driveSubsystem.velocity().omega) < MIN_ROTATE) {
                headingSetpoint = heading;
                headingLocked = true;
            }
        } else {
            headingLocked = false;
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                axial, lateral, yaw, driveSubsystem.pose().heading()
        );

        // Robot-centric drive
        driveSubsystem.manual(powers);
    }

    /**
     * Connect button triggers to commands
     */
    public void bindButtons() {
        GamepadEx driverOp = new GamepadEx(gamepad1);

        // Home the pose (location and heading)
        driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD)
            .whenPressed(new InstantCommand(() -> resetHeading()));

        // Select the alliance cluster to track (hold Right Bumper to point at it)
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(() -> visionSubsystem.setAlliance(false)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(new InstantCommand(() -> visionSubsystem.setAlliance(true)));

        // Turn on flower collector
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(flowerSubsystem.startCommand())
            .whenReleased(flowerSubsystem.stopCommand());

        // Reverse flower collector
        new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
            .whenActive(flowerSubsystem.reverseCommand())
            .whenInactive(flowerSubsystem.stopCommand());
    }

    public void resetHeading () {
        driveSubsystem.setPose(Pose.zero());
        headingSetpoint = 0.0;
    }
}

