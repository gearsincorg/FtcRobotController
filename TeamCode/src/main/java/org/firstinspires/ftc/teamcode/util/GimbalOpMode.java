package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.GimbalSubsystem;

/**
 * Gimbal follows gamepad1's right joystick.
 *   Right stick up / down    : pitch
 *   Right stick left / right : roll
 * Releasing the stick re-centers the gimbal.
 */
@TeleOp(name = "Gimbal", group = "Test")
public class GimbalOpMode extends CommandOpMode {

    private GimbalSubsystem gimbalSubsystem;

    @Override
    public void initialize() {
        gimbalSubsystem = new GimbalSubsystem(this);
        gimbalSubsystem.init();

        // Stick up is negative y, so negate it to make up = +pitch.
        gimbalSubsystem.setDefaultCommand(gimbalSubsystem.followCommand(
                () -> -gamepad1.right_stick_y,
                () ->  gamepad1.right_stick_x));
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
