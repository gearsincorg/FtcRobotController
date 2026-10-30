package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Bench test for the intake state machine.  No drive or shooter needed.
 *
 * The intake collects by itself.  Hold A shoot, hold B eject, hold Y clear all
 * (the teleop buttons, except shoot is A here instead of Right Bumper).
 * Hold Right Bumper to pretend the shooter is ready.
 * D-pad up / down adds / removes a simulated ball (when the sensors are simulated).
 */
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTestOpMode extends CommandOpMode {

    private IntakeSubsystem intake;

    // Previous button states, for edge detection
    private boolean lastUp, lastDown;

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(this, () -> gamepad1.right_bumper);
        intake.init();

        new Trigger(() -> gamepad1.a).whileActiveOnce(intake.shootWhileHeldCommand());
        new Trigger(() -> gamepad1.b).whileActiveOnce(intake.ejectCommand());
        new Trigger(() -> gamepad1.y).whileActiveOnce(intake.clearAllCommand());
    }

    @Override
    public void run() {
        super.run();

        if (gamepad1.dpad_up && !lastUp)     intake.simAddBall();
        if (gamepad1.dpad_down && !lastDown) intake.simRemoveBall();
        lastUp   = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        telemetry.addLine("A shoot, B eject, Y clear all, RB = shooter ready");
        telemetry.addData("Shooter ready", gamepad1.right_bumper);
        if (intake.isSimulated()) {
            telemetry.addLine("D-pad up / down: add / remove a simulated ball");
        }
        telemetry.update();
    }
}
