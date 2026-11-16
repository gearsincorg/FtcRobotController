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
import com.pedropathing.math.Velocity;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Commands.RobotCommands;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp(name = "G-FORCE TELEOP", group = "Competition")
public class GFORCE_Teleop extends CommandOpMode {

    // Declare robot Subsystems
    private BiobuzzRobot robot = new BiobuzzRobot(this);

    // Optional vision features.  Off: the camera does no detection work and the button does nothing
    // extra.  On: the finder runs only while its button (L1 / R2) is held.
    private static final boolean BALL_CHASE_ENABLED   = false;   // hold L1: chase the nearest yellow ball (was for testing; L1 is now Flower Intake)
    private static final boolean CLUSTER_AIM_ENABLED  = false;   // hold R2: turn toward the AprilTag cluster when seen (fallback for goal aim)

    // All turning uses Foresight's heading controller (robot.turnForError / turnToHeading); these
    // only limit the turn power.
    // R2 (fire) and L2 (eject) count as held past this much pull (0..1).
    private static final double TRIGGER_PULL = 0.5;

    private static final double MAX_TARGET_TURN = 0.6;   // point at the target cluster
    private static final double MAX_BALL_TURN   = 0.4;   // chase a ball
    private static final double MAX_AIM_TURN    = 0.6;   // point the shooter at the goal
    private static final double MAX_SNAP_TURN   = 0.6;   // heading lock / face-button heading snap

    // Pose set by the Touchpad (the Auto1 start position).  Heading is in radians.
    private static final Pose RESET_POSE = new Pose(32.75, 6, Math.toRadians(180));

    // Pedro field direction that "stick forward" means to the driver (away from the driver station).
    private static final double DRIVER_FORWARD = Math.toRadians(0);

    // class Methods
    private Constants.Chassis   chassis;
    private boolean             aimAtGoal = false;
    private boolean             aimingActive = false;   // the aim branch drove the turn last loop
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

        // The shooter always tracks the goal: the flywheel is kept at the table speed (and the hood
        // at the table angle) for wherever we are, from START to the end.  Fire only when it's at
        // that speed AND the chassis points at the goal.
        robot.shooter.track();
        robot.shooter.setRequireChassisAim(true);

        setAlliance(robot.config.red);

        // Connect buttons with commands
        bindButtons();
    }

    private void showTelemetry() {
        Pose pose = robot.drive.pose();
        Velocity vel = robot.drive.velocity();
        telemetry.addData("Cycle", "%4.1f ms  (avg %4.1f)", cycleMs, cycleAvgMs);
        telemetry.addData("Pose", "x %5.1f  y %5.1f  hdg %5.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Vel", "x %5.1f  y %5.1f  turn %5.2f", vel.vx, vel.vy, vel.omega);
        telemetry.update();

        showBallCountLed();
    }

    // Loop (cycle) time: the time between run() calls, and a smoothed average.
    private final ElapsedTime cycleTimer = new ElapsedTime();
    private double cycleMs = 0, cycleAvgMs = 0;

    private void timeCycle() {
        cycleMs = cycleTimer.milliseconds();
        cycleTimer.reset();
        cycleAvgMs = cycleAvgMs == 0 ? cycleMs : cycleAvgMs + 0.05 * (cycleMs - cycleAvgMs);
    }

    @Override
    public void run() {
        timeCycle();
        super.run();
        gamepad2SimBalls();
        manualDrive();
        robot.update();
        showTelemetry();
    }

    // == Additional Methods ===================================================

    public void manualDrive() {
        // Manual driving ===================================
        final double DRIVE_SCALE_FACTOR = 1.0;
        final double STEER_SCALE_FACTOR = 0.7;
        final double MIN_ROTATE      = 0.1;

        double axial    = -gamepad1.left_stick_y  * DRIVE_SCALE_FACTOR;
        double lateral  = -gamepad1.left_stick_x  * DRIVE_SCALE_FACTOR;
        double yaw      = -gamepad1.right_stick_x * STEER_SCALE_FACTOR;

        double heading = robot.drive.pose().heading();
        boolean aiming = false;     // set by the aim branch; a new aim resets the heading controller

        if (BALL_CHASE_ENABLED && gamepad1.left_bumper && robot.vision.hasBall()) {
            // Chase the nearest yellow ball (the intake is always collecting).  With the left
            // stick idle, drive straight ahead (robot-forward) at BALL_DRIVE_POWER; pushing it
            // drives field-centric as usual.  Twisting the right stick overrides the turn.
            if (yaw == 0) yaw = robot.turnForError(robot.vision.getBallHeadingErrorRad(), MAX_BALL_TURN);
            headingLocked = false;
            if (axial == 0 && lateral == 0) {
                aimingActive = false;
                robot.drive.manual(ManualDrive.fieldCentric(robot.vision.getBallForwardError(), 0, yaw, 0));
                return;
            }
        } else if (CLUSTER_AIM_ENABLED && fireHeld() && robot.vision.hasTarget()) {
            // Point at the target cluster: use the bearing (+ is CCW) as the heading error.
            yaw = robot.turnForError(robot.vision.getTargetHeadingErrorRad(), MAX_TARGET_TURN);
            headingLocked = false;
        } else if (yaw == 0 && aimAtGoal) {
            // Point the shooter at the goal (leading it when moving).  Twisting the stick overrides.
            // Start each aim with a fresh controller (no memory from earlier turning).
            if (!aimingActive) robot.resetHeadingController();
            yaw = robot.turnForError(robot.shooter.getAimError(), MAX_AIM_TURN);
            aiming = true;
            headingLocked = false;
        } else if (yaw == 0 ) {
            // Lock heading if we aren't trying to turn.
            if (headingLocked) {
                yaw = robot.turnToHeading(headingSetpoint, MAX_SNAP_TURN);
            } else if (Math.abs(robot.drive.velocity().omega) < MIN_ROTATE) {
                headingSetpoint = heading;
                headingLocked = true;
                robot.resetHeadingController();
            }
        } else {
            headingLocked = false;
        }
        aimingActive = aiming;

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
        // Hold R1 to point at the cluster (CLUSTER_AIM_ENABLED only).
        driverOp.getGamepadButton(GamepadKeys.Button.SHARE)
            .whenPressed(new InstantCommand(() -> setAlliance(false)));
        driverOp.getGamepadButton(GamepadKeys.Button.OPTIONS)
            .whenPressed(new InstantCommand(() -> setAlliance(true)));

        // Face buttons turn to a field heading, laid out like a compass on the pad (Triangle up,
        // Square left, Cross down, Circle right).  Twisting the stick takes over.
        driverOp.getGamepadButton(GamepadKeys.Button.TRIANGLE)
            .whenPressed(new InstantCommand(() -> setHeadingTarget(0)));
        driverOp.getGamepadButton(GamepadKeys.Button.SQUARE)
            .whenPressed(new InstantCommand(() -> setHeadingTarget(90)));
        driverOp.getGamepadButton(GamepadKeys.Button.CROSS)
            .whenPressed(new InstantCommand(() -> setHeadingTarget(180)));
        driverOp.getGamepadButton(GamepadKeys.Button.CIRCLE)
            .whenPressed(new InstantCommand(() -> setHeadingTarget(270)));

        // Point-at-goal: once we're full, the chassis turns to the goal by itself; once everything
        // has been shot (or ejected) and fire is released, it stops.  The shooter keeps spinning.
        new Trigger(robot.intake::isFull)
            .whenActive(new InstantCommand(() -> setAimAtGoal(true)));
        new Trigger(() -> robot.intake.isEmpty() && !robot.intake.isShooting())
            .whenActive(new InstantCommand(() -> setAimAtGoal(false)));

        // R3 (right stick press): point-at-goal on / off by hand.
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(new InstantCommand(this::toggleAimAtGoal));

        // The intake collects by itself (and spits out a 5th ball when full).

        // R2: fire while held (pulled past TRIGGER_PULL).  Pulling it also turns point-at-goal on
        // (for a partial load), but no ball is fed until the shooter is ready (at speed and aimed);
        // then it feeds until released.  Overrides Pass and eject.
        // With CLUSTER_AIM_ENABLED, AprilTags run only while R2 is held (manualDrive points at the
        // cluster when it is seen).
        new Trigger(this::fireHeld)
            .whenActive(new InstantCommand(() -> {
                setAimAtGoal(true);
                if (CLUSTER_AIM_ENABLED) robot.vision.enableAprilTags(true);
            }))
            .whenInactive(new InstantCommand(() -> {
                robot.vision.enableAprilTags(false);
                if (robot.intake.isEmpty()) setAimAtGoal(false);
            }))
            .whileActiveOnce(robot.intake.shootWhileHeldCommand());

        // D-pad down: Nectar In, D-pad up: Nectar Out, while held.  Placeholders: not implemented yet,
        // so they do nothing (see nectarInCommand() / nectarOutCommand()).
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whileHeld(nectarInCommand());
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whileHeld(nectarOutCommand());

        // L1: Flower Intake while held.  Placeholder: the mode doesn't exist yet, so it does nothing.
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whileHeld(robot.intake.flowerIntakeCommand());

        // L1 also chases a ball when BALL_CHASE_ENABLED (manualDrive); they would share the button,
        // so give ball chase another one before turning it on.  The ball finder runs only while held.
        if (BALL_CHASE_ENABLED) {
            driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> robot.vision.enableBallFinder(true)))
                .whenReleased(new InstantCommand(() -> robot.vision.enableBallFinder(false)));
        }

        // L2: eject while held (pulled past TRIGGER_PULL): intake out, the transfer holds its balls.
        new Trigger(() -> gamepad1.left_trigger > TRIGGER_PULL)
            .whileActiveOnce(robot.intake.ejectCommand());

        // R1: Pass while held: intake and transfer reversed, every ball goes out the intake.
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whileHeld(robot.intake.passCommand());

        // Driver feedback
        new Trigger(robot.intake::isFull).whenActive(() -> gamepad1.rumble(300));
        new Trigger(robot.intake::isEmpty).whenActive(() -> gamepad1.rumbleBlips(2));
    }

    /** Gamepad LED shows the ball count: red empty, amber partial, green full. */
    private void showBallCountLed() {
        int state = robot.intake.isFull() ? 2 : robot.intake.isEmpty() ? 0 : 1;
        if (state == lastLedCount) return;
        lastLedCount = state;
        if (state == 0) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (state == 2) {
            gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(1, 0.5, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    /** Gamepad 2 D-pad up / down adds / removes a simulated ball. */
    private void gamepad2SimBalls() {
        if (gamepad2.dpad_up && !lastSimUp)     robot.intake.simAddBall();
        if (gamepad2.dpad_down && !lastSimDown) robot.intake.simRemoveBall();
        lastSimUp   = gamepad2.dpad_up;
        lastSimDown = gamepad2.dpad_down;
    }

    public void setAlliance(boolean red) {
        robot.vision.setAlliance(red);
        robot.shooter.setAlliance(red);
    }

    /** R2 pulled far enough to fire. */
    private boolean fireHeld() {
        return gamepad1.right_trigger > TRIGGER_PULL;
    }

    /** Toggle point-at-goal (the chassis turning to aim the shooter). */
    public void toggleAimAtGoal() {
        setAimAtGoal(!aimAtGoal);
    }

    /** Point-at-goal on or off.  The shooter keeps tracking (spinning at the table speed) either way. */
    public void setAimAtGoal(boolean on) {
        aimAtGoal = on;
    }

    /** Nectar In while held.  TODO: not implemented yet (no mechanism or mode); does nothing. */
    private Command nectarInCommand() {
        return new StartEndCommand(() -> {}, () -> {});
    }

    /** Nectar Out while held.  TODO: not implemented yet (no mechanism or mode); does nothing. */
    private Command nectarOutCommand() {
        return new StartEndCommand(() -> {}, () -> {});
    }

    /** Turn to a Pedro field heading (degrees) and hold it until the driver twists the stick. Cancels point-at-goal. */
    public void setHeadingTarget(double degrees) {
        headingSetpoint = Math.toRadians(degrees);
        headingLocked = true;
        robot.resetHeadingController();
        aimAtGoal = false;
    }

    public void resetHeading () {
        robot.drive.setPose(RESET_POSE);
        headingSetpoint = RESET_POSE.heading();
    }
}


