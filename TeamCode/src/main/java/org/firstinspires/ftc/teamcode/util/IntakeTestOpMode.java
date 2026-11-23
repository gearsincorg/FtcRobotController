package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Bench test for the intake state machine.  No drive or shooter needed.
 *
 * The intake collects by itself.  Hold Cross shoot, hold Circle eject, hold Triangle pass
 * (the teleop buttons, except shoot is Cross here instead of R1).
 * Hold R1 to pretend the shooter is ready.
 * D-pad up / down adds / removes a simulated ball (when the sensors are simulated).
 */
@Disabled
@Utility(name = "Intake Test", description = "Test")
public class IntakeTestOpMode extends CommandOpMode {

    private IntakeSubsystem intake;

    // Previous button states, for edge detection
    private boolean lastUp, lastDown;

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(this, () -> gamepad1.right_bumper);
        intake.init();
        intake.setShowPowers(true);

        new Trigger(() -> gamepad1.cross).whileActiveOnce(intake.shootWhileHeldCommand());
        new Trigger(() -> gamepad1.circle).whileActiveOnce(intake.ejectCommand());
        new Trigger(() -> gamepad1.triangle).whileActiveOnce(intake.passCommand());
    }

    @Override
    public void run() {
        super.run();

        if (gamepad1.dpad_up && !lastUp)     intake.simAddBall();
        if (gamepad1.dpad_down && !lastDown) intake.simRemoveBall();
        lastUp   = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        telemetry.addLine("Cross shoot, Circle eject, Triangle pass, R1 = shooter ready");
        telemetry.addData("Shooter ready", gamepad1.right_bumper);
        if (intake.isSimulated()) {
            telemetry.addLine("D-pad up / down: add / remove a simulated ball");
        }
        telemetry.update();
    }
}
