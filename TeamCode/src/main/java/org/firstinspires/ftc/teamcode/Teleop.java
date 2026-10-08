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
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "G-FORCE TELEOP", group = "Sensor")
public class Teleop extends CommandOpMode {

    private FlowerIntake        flowerIntake;

    private Follower            follower;
    private boolean             slowMode = false;
    private boolean             headingLocked = false;
    private double              headingSetpoint = 0;

    @Override
    public void initialize() {
        follower = Constants.create(hardwareMap);
        flowerIntake = new FlowerIntake(this);
        CommandScheduler.getInstance().reset();
        bindButtons();
         // super.reset();  // Resets the scheduler (I think :)
    }

    @Override
    public void run() {
        super.run();

        // Manual driving ===================================
        final double SLOW_MULTIPLIER = 0.5;
        final double HEADING_GAIN    = 1.0;
        final double MIN_ROTATE      = 0.1;

        double axial    = -gamepad1.left_stick_y  * SLOW_MULTIPLIER;
        double lateral  = -gamepad1.left_stick_x  * SLOW_MULTIPLIER;
        double yaw      = -gamepad1.right_stick_x * SLOW_MULTIPLIER;

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

        DrivePowers powers = ManualDrive.fieldCentric(
            axial, lateral, yaw, follower.pose().heading()
        );

        // Robot-centric drive
        follower.manual(powers);

        // Main Pedro and Solver loop processing.
        follower.update();
        CommandScheduler.getInstance().run();

        //telemetry.addData("pos X: Y", "%5.1f : %5.1f", follower.pose().x(), follower.pose().y());
       // telemetry.addData("vel X: Y: O", "%5.1f : %5.1f : %5.0f", follower.velocity().vx,  follower.velocity().vy,  follower.velocity().omega);
        //telemetry.update();
    }

    public void resetHeading () {
        follower.setPose(Pose.zero());
        headingSetpoint = 0.0;
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    /**
     * Connect button triggers to commands
     */
    public void bindButtons() {
        GamepadEx driverOp = new GamepadEx(gamepad1);

        // Home the pose (location and heading)
        driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD)
            .whenPressed(new InstantCommand(() -> resetHeading()));

        // Turn on flower collector
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(flowerIntake.onCommand())
            .whenReleased(flowerIntake.offCommand());

        // Reverse flower collector
        Trigger leftTriggerSwitch = new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        leftTriggerSwitch.whenActive(flowerIntake.offCommand());

        new GamepadTrigger(driverOp, GamepadKeys.Trigger.LEFT_TRIGGER)
            .whenPressed(flowerIntake.reverseCommand())
            .whenReleased(flowerIntake.offCommand());

        // Toggle slow mode
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(new InstantCommand(() -> toggleSlowMode()));
    }
}

class GamepadTrigger extends GamepadButton {

    GamepadEx           driverOp;
    GamepadKeys.Trigger triger;

    public GamepadTrigger(GamepadEx driverOp, GamepadKeys.Trigger triger){
        super(driverOp);
        this.driverOp = driverOp;
        this.triger = triger;
    }

    @Override
    public boolean get() {
        return driverOp.getTrigger(triger) > 0.5;
    }
}
