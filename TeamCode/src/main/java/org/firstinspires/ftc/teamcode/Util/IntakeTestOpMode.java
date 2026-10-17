package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

/**
 * Bench test for the intake, gate and feeder.  No drive or shooter needed.
 *
 * Back / Share switches pages:
 *   RAW            left stick Y = intake power, right stick Y = feeder power
 *                  X = gate closed, Y = gate open, D-pad left/right = nudge gate by 0.01
 *                  (use this to find IntakeSubsystem.GATE_OPEN and GATE_CLOSED)
 *   STATE MACHINE  the teleop buttons: hold LB intake, A shoot all, RT shoot one, hold B eject.
 *                  Hold Right Bumper to pretend the shooter is ready.
 * D-pad up / down adds / removes a simulated ball (when the sensors are simulated).
 */
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTestOpMode extends CommandOpMode {

    private static final double GATE_STEP = 0.01;

    private IntakeSubsystem intake;
    private boolean statePage = false;
    private double  gatePosition = IntakeSubsystem.GATE_CLOSED;

    // Previous button states, for edge detection
    private boolean lastBack, lastLeft, lastRight, lastUp, lastDown;

    @Override
    public void initialize() {
        intake = new IntakeSubsystem(this, () -> gamepad1.right_bumper);
        intake.init();

        // State machine page: the same buttons as teleop.
        new Trigger(() -> statePage && gamepad1.left_bumper)
            .whileActiveOnce(intake.intakeCommand());
        new Trigger(() -> statePage && gamepad1.a)
            .whenActive(intake.shootAllCommand());
        new Trigger(() -> statePage && gamepad1.right_trigger > 0.5)
            .whenActive(intake.shootOneCommand());
        new Trigger(() -> statePage && gamepad1.b)
            .whileActiveOnce(intake.ejectCommand());
    }

    @Override
    public void run() {
        super.run();

        if (gamepad1.back && !lastBack) {
            statePage = !statePage;
            if (statePage) {
                intake.clearManual();
            }
        }

        if (gamepad1.dpad_up && !lastUp)     intake.simAddBall();
        if (gamepad1.dpad_down && !lastDown) intake.simRemoveBall();

        if (!statePage) {
            if (gamepad1.x) gatePosition = IntakeSubsystem.GATE_CLOSED;
            if (gamepad1.y) gatePosition = IntakeSubsystem.GATE_OPEN;
            if (gamepad1.dpad_left && !lastLeft)   gatePosition -= GATE_STEP;
            if (gamepad1.dpad_right && !lastRight) gatePosition += GATE_STEP;
            gatePosition = Range.clip(gatePosition, 0, 1);

            intake.setManual(-gamepad1.left_stick_y, -gamepad1.right_stick_y, gatePosition);
        }

        lastBack  = gamepad1.back;
        lastLeft  = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastUp    = gamepad1.dpad_up;
        lastDown  = gamepad1.dpad_down;

        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addData("Page", statePage ? "STATE MACHINE (Back = raw)" : "RAW (Back = state machine)");
        if (statePage) {
            telemetry.addLine("LB intake, A shoot all, RT shoot one, B eject, RB = shooter ready");
            telemetry.addData("Shooter ready", gamepad1.right_bumper);
        } else {
            telemetry.addLine("Sticks: intake / feeder.  X closed, Y open, D-pad L/R nudge gate");
            telemetry.addData("Intake : feeder", "%4.2f : %4.2f",
                    -gamepad1.left_stick_y, -gamepad1.right_stick_y);
            telemetry.addData("Gate position", "%4.2f  (closed %4.2f, open %4.2f)",
                    gatePosition, IntakeSubsystem.GATE_CLOSED, IntakeSubsystem.GATE_OPEN);
        }
        if (intake.isSimulated()) {
            telemetry.addLine("D-pad up / down: add / remove a simulated ball");
        }
        telemetry.update();
    }
}
