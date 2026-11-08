package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.BiobuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

/**
 * Test for yellow ball tracking while intaking (Webcam 1, shared with AprilTags).
 *
 * The intake collects by itself.  Drive robot-centric with the sticks.
 *   LB     hold to drive forward at 20% speed (BALL_DRIVE_POWER) and turn toward the nearest yellow ball (the one
 *          lowest in the image); the sticks override.  Released = normal driving.
 *   B      eject (hold), Y clear all (hold)
 * Telemetry "BALL" shows the ball's side (LEFT / CENTER / RIGHT), its pixel position, how far off
 * center it is (-1 left edge .. +1 right edge) and its size.  The camera stream draws a rectangle
 * around it.  The gamepad LED is yellow while a ball is seen.
 * The color range is BiobuzzRobot.POLLEN_YELLOW.
 */
@TeleOp(name = "Color Tracking Test", group = "Test")
public class ColorTrackingTestOpMode extends CommandOpMode {

    // Chase speed and turn tuning are in VisionSubsystem (BALL_DRIVE_POWER, BALL_TURN_GAIN, MAX_BALL_TURN).
    private static final double DRIVE_SCALE = 0.7;

    private final BiobuzzRobot robot = new BiobuzzRobot(this);
    private VisionSubsystem vision;
    private Boolean lastLedOn = null;

    @Override
    public void initialize() {
        robot.initialize();
        vision = robot.vision;

        GamepadEx driverOp = new GamepadEx(gamepad1);
        driverOp.getGamepadButton(GamepadKeys.Button.B)
            .whileHeld(robot.intake.ejectCommand());
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
            .whileHeld(robot.intake.clearAllCommand());
    }

    @Override
    public void run() {
        super.run();
        drive();
        robot.update();
        showLed();

        telemetry.addLine("Hold LB drive to ball, B eject, Y clear all");
        telemetry.addData("Steering", gamepad1.left_bumper ? "ON" : "off");
        telemetry.update();
    }

    private void drive() {
        double axial   = -gamepad1.left_stick_y  * DRIVE_SCALE;
        double lateral = -gamepad1.left_stick_x  * DRIVE_SCALE;
        double yaw     = -gamepad1.right_stick_x * DRIVE_SCALE;

        // Hold Left Bumper to drive forward and steer to the ball.  The sticks override.
        if (gamepad1.left_bumper) {
            if (axial == 0) axial = VisionSubsystem.BALL_DRIVE_POWER;
            if (yaw == 0)   yaw   = vision.getBallTurn();
        }

        // A heading of 0 makes it robot-centric.
        DrivePowers powers = ManualDrive.fieldCentric(axial, lateral, yaw, 0);
        robot.drive.manual(powers);
    }

    /** LED is yellow while a ball is seen, otherwise off. */
    private void showLed() {
        boolean on = vision.hasBall();
        if (lastLedOn != null && lastLedOn == on) return;
        lastLedOn = on;
        if (on) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}
