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
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "G-FORCE TELEOP", group = "Competition")
public class GFORCE_Teleop extends CommandOpMode {

    // Declare robot Subsystems
    private Robot               robot = new Robot(this);

    // Point-at-target tuning
    private static final double TARGET_TURN_GAIN = 0.04;
    private static final double MAX_TARGET_TURN  = 0.6;

    // Point-at-goal tuning (no turret: the chassis turns to aim the shooter)
    private static final double AIM_GAIN     = 0.5;   // power per radian of aim error
    private static final double MAX_AIM_TURN = 0.6;

    // Heading lock / D-pad heading snap tuning
    private static final double MAX_SNAP_TURN = 0.6;

    // Pose set by the Touchpad (the Auto1 start position).  Heading is in radians.
    private static final Pose RESET_POSE = new Pose(56, 9, Math.toRadians(90));

    // Pedro field direction that "stick forward" means to the driver (away from the driver station).
    private static final double DRIVER_FORWARD = Math.toRadians(90);

    // class Methods
    private Constants.Chassis   chassis;
    private boolean             aimAtGoal = false;
    private boolean             headingLocked = false;
    private double              headingSetpoint = 0;
    private int                 lastLedCount = -1;
    private boolean             lastSimUp, lastSimDown;


    // == Standard Methods ===================================================
    @Override
    public void initialize() {
        // Create subsystems here (not as field initializers) so they register with the
        chassis = Constants.chassis();

        robot.initialize();

        setAlliance(robot.config.red);

        // Connect buttons with commands
        bindButtons();
    }

    private void showTelemetry() {
        telemetry.addData("chassis", chassis);
        telemetry.addData("aim at goal", "%s  error = %5.1f deg", aimAtGoal ? "ON" : "off",
                            Math.toDegrees(robot.shooter.getAimError()));
        telemetry.addData("pos X: Y", "%5.1f : %5.1f", robot.drive.pose().x(), robot.drive.pose().y());
        telemetry.addData("heading", "%5.1f deg", Math.toDegrees(robot.drive.pose().heading()));
        telemetry.addData("vel X: Y: O", "%5.1f : %5.1f : %5.0f", robot.drive.velocity().vx,
                robot.drive.velocity().vy,  robot.drive.velocity().omega);
        telemetry.update();

        showBallCountLed();
    }

    @Override
    public void run() {
        super.run();
        gamepad2Intake();
        manualDrive();
        robot.update();
        showTelemetry();
    }

    // == Additional Methods ===================================================

    public void manualDrive() {
        // Manual driving ===================================
        final double SLOW_MULTIPLIER = 0.7;
        final double HEADING_GAIN    = 0.5;
        final double MIN_ROTATE      = 0.1;

        double axial    = -gamepad1.left_stick_y  * SLOW_MULTIPLIER;
        double lateral  = -gamepad1.left_stick_x  * SLOW_MULTIPLIER;
        double yaw      = -gamepad1.right_stick_x * SLOW_MULTIPLIER;

        double heading = robot.drive.pose().heading();

        if (gamepad1.right_bumper && robot.vision.hasTarget()) {
            // Point at the target cluster: use the bearing (+ is CCW) as the error.
            yaw = Range.clip(robot.vision.getTargetPose().bearing * TARGET_TURN_GAIN,
                             -MAX_TARGET_TURN, MAX_TARGET_TURN);
            headingLocked = false;
        } else if (yaw == 0 && aimAtGoal) {
            // Point the shooter at the goal (leading it when moving).  Twisting the stick overrides.
            yaw = Range.clip(robot.shooter.getAimError() * AIM_GAIN, -MAX_AIM_TURN, MAX_AIM_TURN);
            headingLocked = false;
        } else if (yaw == 0 ) {
            // Lock heading if we aren't trying to turn.
            if (headingLocked) {
                yaw = Range.clip(Angle.normalizeSigned(headingSetpoint - heading) * HEADING_GAIN,
                                 -MAX_SNAP_TURN, MAX_SNAP_TURN);
            } else if (Math.abs(robot.drive.velocity().omega) < MIN_ROTATE) {
                headingSetpoint = heading;
                headingLocked = true;
            }
        } else {
            headingLocked = false;
        }

        // Field centric, relative to the driver's view rather than Pedro's +X axis.
        DrivePowers powers = ManualDrive.fieldCentric(
                axial, lateral, yaw, heading - DRIVER_FORWARD
        );

        // Robot-centric drive
        robot.drive.manual(powers);
    }

    /**
     * Connect button triggers to commands
     */
    public void bindButtons() {
        GamepadEx driverOp = new GamepadEx(gamepad1);

        // Home the pose (location and heading)
        driverOp.getGamepadButton(GamepadKeys.Button.TOUCHPAD)
            .whenPressed(new InstantCommand(() -> resetHeading()));

        // Select the alliance (vision cluster and shooter target): Share = blue, Options = red.
        // Hold Right Bumper to point at the cluster.
        driverOp.getGamepadButton(GamepadKeys.Button.BACK)
            .whenPressed(new InstantCommand(() -> setAlliance(false)));
        driverOp.getGamepadButton(GamepadKeys.Button.START)
            .whenPressed(new InstantCommand(() -> setAlliance(true)));

        // D-pad turns to a field heading (as the driver sees it).  Twisting the stick takes over.
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(new InstantCommand(() -> setHeadingTarget(90)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(new InstantCommand(() -> setHeadingTarget(180)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(new InstantCommand(() -> setHeadingTarget(270)));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(new InstantCommand(() -> setHeadingTarget(0)));

        // The shooter spins up and aims by itself when we're full, and spins down when empty.
        RobotCommands.bindAutoAim(robot.shooter, robot.intake);

        // Point-at-goal: on by itself when full, off when empty; Right Stick press toggles it,
        // and starts or stops the shooter with it.
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(new InstantCommand(() -> toggleAimAtGoal()));
        new Trigger(robot.intake::isFull)
            .whenActive(() -> aimAtGoal = true);
        new Trigger(() -> !IntakeSubsystem.TIMED_SHOTS
                && robot.intake.isEmpty() && !robot.intake.isShooting())
            .whenActive(() -> aimAtGoal = false);

        // A: fire every ball (each waits for the shooter to be ready).  Double rumble when done.
        driverOp.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(RobotCommands.shootAll(robot.shooter, robot.intake)
                .andThen(new InstantCommand(() -> {
                    if (robot.intake.isEmpty()) gamepad1.rumbleBlips(2);
                })));

        // Right Trigger: fire one ball per press
        new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
            .whenActive(RobotCommands.shootOne(robot.shooter, robot.intake));

        // Left Bumper: collect while held (stops and spits out extras by itself when full)
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whileHeld(robot.intake.intakeCommand());

        // B: eject every ball out of the intake while held
        driverOp.getGamepadButton(GamepadKeys.Button.B)
            .whileHeld(robot.intake.ejectCommand());

        // Driver feedback
        new Trigger(robot.intake::isFull).whenActive(() -> gamepad1.rumble(300));
        new Trigger(robot.intake::isJammed).whenActive(() -> gamepad1.rumble(1000));
    }

    /** Gamepad LED shows the ball count: red empty, amber partial, green full. */
    private void showBallCountLed() {
        int count = robot.intake.ballCount();
        if (count == lastLedCount) return;
        lastLedCount = count;
        if (count == 0) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (robot.intake.isFull()) {
            gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(1, 0.5, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    /**
     * Gamepad 2 fallback if the sensors fail: left stick = intake, right stick = feeder,
     * hold X = gate open.  Letting go hands control back to the state machine.
     * D-pad up / down adds / removes a simulated ball.
     */
    private void gamepad2Intake() {
        double intakePower = -gamepad2.left_stick_y;
        double feederPower = -gamepad2.right_stick_y;
        if (Math.abs(intakePower) > 0.1 || Math.abs(feederPower) > 0.1 || gamepad2.x) {
            robot.intake.setManual(intakePower, feederPower,
                    gamepad2.x ? IntakeSubsystem.GATE_OPEN : IntakeSubsystem.GATE_CLOSED);
        } else {
            robot.intake.clearManual();
        }

        if (gamepad2.dpad_up && !lastSimUp)     robot.intake.simAddBall();
        if (gamepad2.dpad_down && !lastSimDown) robot.intake.simRemoveBall();
        lastSimUp   = gamepad2.dpad_up;
        lastSimDown = gamepad2.dpad_down;
    }

    public void setAlliance(boolean red) {
        robot.vision.setAlliance(red);
        robot.shooter.setAlliance(red);
    }

    /** Toggle point-at-goal, spinning the shooter up (tracking) or down to match. */
    public void toggleAimAtGoal() {
        aimAtGoal = !aimAtGoal;
        if (aimAtGoal) {
            robot.shooter.track();
        } else {
            robot.shooter.idle();
        }
    }

    /** Turn to a Pedro field heading (degrees) and hold it until the driver twists the stick. Cancels point-at-goal. */
    public void setHeadingTarget(double degrees) {
        headingSetpoint = Math.toRadians(degrees);
        headingLocked = true;
        aimAtGoal = false;
    }

    public void resetHeading () {
        robot.drive.setPose(RESET_POSE);
        headingSetpoint = RESET_POSE.heading();
    }
}


